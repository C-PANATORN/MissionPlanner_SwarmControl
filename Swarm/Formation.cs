using System;
using System.Collections.Generic;
using MissionPlanner.ArduPilot;
using MissionPlanner.Utilities;
using ProjNet.CoordinateSystems;
using ProjNet.CoordinateSystems.Transformations;
using GeoAPI.CoordinateSystems;
using GeoAPI.CoordinateSystems.Transformations;
using MathNet.Numerics.LinearAlgebra;
using Vector3 = MissionPlanner.Utilities.Vector3;

namespace MissionPlanner.Swarm
{
    /// <summary>
    /// Formation control with Flatness-Based Adaptive Control (FBAC):
    /// analytic feed-forward, adaptive unified feedback, wrench-based tension distribution with null-space balancing, and yaw feed-forward.
    /// </summary>
    internal class Formation : Swarm
    {
        // Relative offsets
        private readonly Dictionary<MAVState, Vector3> offsets = new Dictionary<MAVState, Vector3>();
        // Smoothed tension compensation
        private readonly Dictionary<MAVState, Vector3> compFiltered = new Dictionary<MAVState, Vector3>();
        // Yaw rate history
        private readonly Dictionary<MAVState, float> prevYawRate = new Dictionary<MAVState, float>();
        // Payload wrench solver
        private readonly LoadAttitudeController payloadController = new LoadAttitudeController();

        // Adaptive parameters
        private float payloadGain = 1.0f;
        private float Kp = 2.0f;
        private float Kv = 1.0f;
        // Adaptation rates
        private const float gammaPayload = 0.1f;
        private const float gammaKp = 0.05f;
        private const float gammaKv = 0.05f;
        // Tension smoothing time constant
        private const float compTau = 0.1f;

        private readonly CoordinateTransformationFactory ctfac = new CoordinateTransformationFactory();
        private readonly IGeographicCoordinateSystem wgs84 = GeographicCoordinateSystem.WGS84;
        private PointLatLngAlt masterpos;
        private DateTime lastUpdateTime = DateTime.UtcNow;

        public void setOffsets(MAVState mav, double x, double y, double z) =>
            offsets[mav] = new Vector3((float)x, (float)y, (float)z);
        public Vector3 getOffsets(MAVState mav) => offsets.ContainsKey(mav) ? offsets[mav] : Vector3.Zero;

        public override void Update()
        {
            var mav0 = MainV2.comPort.MAV;
            if (mav0.cs.lat == 0 || mav0.cs.lng == 0) return;
            if (Leader == null) Leader = mav0;
            masterpos = new PointLatLngAlt(Leader.cs.lat, Leader.cs.lng, Leader.cs.alt, "");
            lastUpdateTime = DateTime.UtcNow;
        }

        private double wrap_180(double a) => a > 180 ? a - 360 : a < -180 ? a + 360 : a;

        public override void SendCommand()
        {
            if (Leader == null || masterpos.Lat == 0) return;

            int zone = (int)((masterpos.Lng + 180.0) / 6.0);
            var utm = ProjectedCoordinateSystem.WGS84_UTM(zone, masterpos.Lat >= 0);
            var trans = ctfac.CreateFromCoordinateSystems(wgs84, utm);
            var leadXY = trans.MathTransform.Transform(new[] { masterpos.Lng, masterpos.Lat });
            Vector3 leaderUTM = new Vector3((float)leadXY[0], (float)leadXY[1], (float)masterpos.Alt);

            foreach (var port in MainV2.Comports)
                foreach (var mav in port.MAVlist)
                {
                    if (mav == Leader) continue;

                    // Compute desired formation position with curvature-aware look-ahead
                    Vector3 off = getOffsets(mav);
                    float yawRad = (float)(Leader.cs.yaw * Math.PI / 180.0);
                    float yawRate = payloadController.EstimateYawRate(Leader);
                    float dt = (float)(DateTime.UtcNow - lastUpdateTime).TotalSeconds;
                    float deltaYaw = yawRate * dt;
                    float cWarp = (float)Math.Cos(deltaYaw), sWarp = (float)Math.Sin(deltaYaw);
                    Vector3 offWarp = new Vector3(
                        off.x * cWarp - off.y * sWarp,
                        off.x * sWarp + off.y * cWarp,
                        off.z);
                    float cYaw = (float)Math.Cos(yawRad), sYaw = (float)Math.Sin(yawRad);
                    Vector3 offRot = new Vector3(
                        offWarp.x * cYaw - offWarp.y * sYaw,
                        offWarp.x * sYaw + offWarp.y * cYaw,
                        offWarp.z);

                    // desired UTM target
                    Vector3 targetUTM = new Vector3(
                        leaderUTM.x + offRot.x,
                        leaderUTM.y + offRot.y,
                        (float)masterpos.Alt + offRot.z);
                    double[] inv = trans.MathTransform.Inverse().Transform(new[] { targetUTM.x, targetUTM.y });
                    var targetGeo = new PointLatLngAlt(inv[1], inv[0], targetUTM.z, "");

                    if (mav.cs.firmware == Firmwares.ArduPlane)
                    {
                        DateTime now = DateTime.UtcNow;
                        dt = (float)(now - lastUpdateTime).TotalSeconds;

                        // §3.2 Feed‑forward acceleration
                        Vector3 aL = new Vector3(Leader.cs.ax, Leader.cs.ay, Leader.cs.az);
                        float yawAcc = prevYawRate.TryGetValue(mav, out var pr) ? (yawRate - pr) / dt : 0f;
                        prevYawRate[mav] = yawRate;
                        Vector3 a_cent = new Vector3(-yawRate * yawRate * off.x, -yawRate * yawRate * off.y, 0f);
                        Vector3 a_tan = new Vector3(-off.y * yawAcc, off.x * yawAcc, 0f);
                        Vector3 a_ff = aL + a_cent + a_tan;

                        // §3.3 Feedback acceleration
                        var curXY = trans.MathTransform.Transform(new[] { mav.cs.lng, mav.cs.lat });
                        Vector3 curUTM = new Vector3((float)curXY[0], (float)curXY[1], mav.cs.alt);
                        Vector3 e_pos = targetUTM - curUTM;
                        Vector3 curV = new Vector3(mav.cs.vx, mav.cs.vy, mav.cs.vz);
                        Vector3 v_ff = new Vector3(Leader.cs.vx, Leader.cs.vy, Leader.cs.vz)
                                      + new Vector3(-yawRate * off.y, yawRate * off.x, 0f);
                        Vector3 e_vel = v_ff - curV;
                        Vector3 a_fb = e_pos * Kp + e_vel * Kv;

                        // §3.1 Null‑space tension balancing
                        Vector3 partComp = payloadController.CompensateRigidBodyDynamics(Leader, mav, offsets);
                        float alpha = dt / (compTau + dt);
                        Vector3 prevC = compFiltered.ContainsKey(mav) ? compFiltered[mav] : Vector3.Zero;
                        Vector3 smooth = prevC + (partComp - prevC) * alpha;
                        compFiltered[mav] = smooth;

                        // Total commanded acceleration
                        Vector3 u = a_ff + a_fb + smooth * payloadGain;

                        // Consensus-based tightening
                        // Each follower nudges toward neighbors to maintain spacing
                        const float Kc = 0.2f;  // consensus gain
                        Vector3 consensusDelta = Vector3.Zero;
                        int neighborCount = 0;
                        foreach (var peer in port.MAVlist)
                        {
                            if (peer == Leader || peer == mav) continue;
                            // peer's UTM position
                            var pxy = trans.MathTransform.Transform(new[] { peer.cs.lng, peer.cs.lat });
                            Vector3 peerUTM = new Vector3((float)pxy[0], (float)pxy[1], peer.cs.alt);
                            consensusDelta += (peerUTM - curUTM);
                            neighborCount++;
                        }
                        if (neighborCount > 0)
                        {
                            consensusDelta = consensusDelta * (Kc / neighborCount);
                            u += consensusDelta;
                        }

                        // §3.4 Attitude & thrust inversion
                        double phi = Math.Asin(u.y / 9.81) * (180.0 / Math.PI);
                        double tht = Math.Asin(-u.x / 9.81) * (180.0 / Math.PI);
                        float thrust = (float)Math.Max(0.1, Math.Min(1, (u.z + 9.81) / 9.81));

                        // §3.3 Adaptive updates
                        float tensionErr = (float)(e_pos.x * smooth.x + e_pos.y * smooth.y + e_pos.z * smooth.z);
                        float posErrSq = (float)(e_pos.x * e_pos.x + e_pos.y * e_pos.y + e_pos.z * e_pos.z);
                        float velErrSq = (float)(e_vel.x * e_vel.x + e_vel.y * e_vel.y + e_vel.z * e_vel.z);
                        payloadGain += gammaPayload * tensionErr * dt;
                        payloadGain = Math.Max(0f, payloadGain);
                        Kp += gammaKp * posErrSq * dt;
                        Kp = Math.Max(0.1f, Kp);
                        Kv += gammaKv * velErrSq * dt;
                        Kv = Math.Max(0.1f, Kv);

                        // Yaw feed‑forward + feedback
                        double psi_ff = Leader.cs.yaw + yawRate * dt;
                        double psi_err = wrap_180(psi_ff - mav.cs.yaw);
                        var quat = Quaternion.from_euler312((float)(phi * Math.PI / 180.0f), (float)(tht * Math.PI / 180.0f), (float)(psi_err * Math.PI / 180.0f));
                        var msg = new MAVLink.mavlink_set_attitude_target_t
                        {
                            target_system = mav.sysid,
                            target_component = mav.compid,
                            type_mask = 0b10000101 | 0b01000000,
                            q = new float[] { (float)quat.q1, (float)quat.q2, (float)quat.q3, (float)quat.q4 },
                            thrust = thrust
                        };
                        port.sendPacket(msg, mav.sysid, mav.compid);
                        lastUpdateTime = now;
                    }
                    else
                    {
                        var v = new Vector3(Leader.cs.vx, Leader.cs.vy, Leader.cs.vz);
                        port.setPositionTargetGlobalInt(mav.sysid, mav.compid, true, true, false, false,
                            MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT_INT,
                            targetGeo.Lat, targetGeo.Lng, targetGeo.Alt,
                            v.x, v.y, v.z, 0, 0);
                    }
                }
        }
    }

    /// <summary>PID controller</summary>
    internal class PID
    {
        private float dt, input, deriv, integrator, imax, filtHz;
        private readonly float kp, ki, kd;
        private bool resetFilter = true;
        public PID(float p, float i, float d, float im, float fHz, float dt0, float ff)
        { kp = p; ki = i; kd = d; imax = Math.Abs(im); filtHz = Math.Max(fHz, 0.01f); dt = dt0; }
        public void set_input_filter_all(float inVal)
        {
            if (float.IsInfinity(inVal)) return;
            if (resetFilter) { input = inVal; deriv = 0; resetFilter = false; }
            float alpha = dt / (dt + 1f / (2f * (float)Math.PI * filtHz));
            float delta = alpha * (inVal - input);
            input += delta; deriv = dt > 0 ? delta / dt : 0;
        }
        public float get_pid() { float P = kp * input; integrator = Math.Max(-imax, Math.Min(imax, integrator + ki * input * dt)); float D = kd * deriv; return P + integrator + D; }
    }

    internal static class VectorUtils
    {
        public static Vector3 NormalizeVector(Vector3 v)
        { double len = Math.Sqrt(v.x * v.x + v.y * v.y + v.z * v.z); return len == 0 ? Vector3.Zero : new Vector3((float)(v.x / len), (float)(v.y / len), (float)(v.z / len)); }
    }

    /// <summary>Payload wrench solver with null-space balancing</summary>
    internal class LoadAttitudeController
    {
        public double PayloadMass { get; set; } = 1.0;
        private static readonly Matrix<double> JL = Matrix<double>.Build.DenseOfArray(new double[,] { { 0.021, 0, 0 }, { 0, 0.0187, 0 }, { 0, 0, 0.0397 } });
        private static readonly Dictionary<int, Tuple<float, float>> YawHistory = new Dictionary<int, Tuple<float, float>>();
        public float EstimateYawRate(MAVState leader)
        { int id = leader.sysid; float cy = leader.cs.yaw; float ct = (float)(DateTime.UtcNow - new DateTime(1970, 1, 1)).TotalSeconds; if (YawHistory.ContainsKey(id)) { var (py, pt) = YawHistory[id]; float dy = (float)Wrap(cy - py); float dt = ct - pt; YawHistory[id] = Tuple.Create(cy, ct); return dt > 1e-3f ? dy / dt : 0f; } YawHistory[id] = Tuple.Create(cy, ct); return 0f; }
        public Vector3 CompensateRigidBodyDynamics(MAVState leader, MAVState follower, Dictionary<MAVState, Vector3> offsets)
        { if (!offsets.ContainsKey(follower)) return Vector3.Zero; var pts = new List<Vector3>(offsets.Values); if (pts.Count < 3) return Vector3.Zero; var rOff = offsets[follower]; float yawR = EstimateYawRate(leader); float compTc = Math.Abs(yawR) > 10 ? 0.05f : 0.2f; float dt = 0.05f; float alpha = dt / (compTc + dt); float m = (float)PayloadMass; var accel = Vector<double>.Build.DenseOfArray(new double[] { leader.cs.ax * m, leader.cs.ay * m, (leader.cs.az + 9.81f) * m }); var grav = Vector<double>.Build.Dense(new[] { 0.0, 0.0, -9.81 }) * m; var R = BuildRotationMatrix(leader.cs.roll, leader.cs.pitch, leader.cs.yaw); var Wf = -R.Transpose() * (accel + grav); var omega = Vector<double>.Build.Dense(3); var omegaDot = Vector<double>.Build.Dense(3); var Wm = -(JL * omegaDot + Cross(omega, JL * omega)); var W = Vector<double>.Build.Dense(6); W.SetSubVector(0, 3, Wf); W.SetSubVector(3, 3, Wm); var Phi = Matrix<double>.Build.Dense(6, pts.Count); for (int i = 0; i < pts.Count; i++) { var r = pts[i]; var qi = VectorUtils.NormalizeVector(r); Phi[0, i] = qi.x; Phi[1, i] = qi.y; Phi[2, i] = qi.z; Phi[3, i] = r.y * qi.z - r.z * qi.y; Phi[4, i] = r.z * qi.x - r.x * qi.z; Phi[5, i] = r.x * qi.y - r.y * qi.x; } var pinv = Phi.PseudoInverse(); var Tpart = pinv * W; var N = Matrix<double>.Build.DenseIdentity(pts.Count) - pinv * Phi; double avgT = Tpart.Sum() / pts.Count; var equal = Vector<double>.Build.Dense(pts.Count, avgT); var Tnull = N * equal; var Tfinal = Tpart + Tnull; int idx = new List<MAVState>(offsets.Keys).IndexOf(follower); if (idx >= 0 && idx < Tfinal.Count) { var qi = VectorUtils.NormalizeVector(rOff); return qi * (float)Math.Max(0, Tfinal[idx]) * alpha; } return Vector3.Zero; }
        private static double Wrap(double a) => a > 180 ? a - 360 : a < -180 ? a + 360 : a;
        private static Matrix<double> BuildRotationMatrix(float roll, float pitch, float yaw) { double r = roll * (Math.PI / 180.0), p = pitch * (Math.PI / 180.0), y = yaw * (Math.PI / 180.0); double cr = Math.Cos(r), sr = Math.Sin(r), cp = Math.Cos(p), sp = Math.Sin(p), cy = Math.Cos(y), sy = Math.Sin(y); return Matrix<double>.Build.DenseOfArray(new double[,] { { cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr }, { sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr }, { -sp, cp * sr, cp * cr } }); }
        private static Vector<double> Cross(Vector<double> a, Vector<double> b) => Vector<double>.Build.DenseOfArray(new double[] { a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0] });
    }
}
