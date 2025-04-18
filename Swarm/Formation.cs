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
    /// Formation control with FBAC + consensus term (§3.1–3.4 + consensus)
    /// </summary>
    internal class Formation : Swarm
    {
        private readonly Dictionary<MAVState, Vector3> offsets = new Dictionary<MAVState, Vector3>();
        private readonly Dictionary<MAVState, Vector3> compFiltered = new Dictionary<MAVState, Vector3>();
        private readonly Dictionary<MAVState, float> prevYawRate = new Dictionary<MAVState, float>();
        private readonly LoadAttitudeController payloadController = new LoadAttitudeController();

        // Adaptive params
        private float payloadGain = 1.0f;
        private float Kp = 2.0f, Kv = 1.0f;
        private const float gammaPayload = 0.1f, gammaKp = 0.05f, gammaKv = 0.05f;
        private const float compTau = 0.05f;

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
            double[] lXY = trans.MathTransform.Transform(new[] { masterpos.Lng, masterpos.Lat });
            Vector3 leaderUTM = new Vector3((float)lXY[0], (float)lXY[1], (float)masterpos.Alt);

            foreach (var port in MainV2.Comports)
                foreach (var mav in port.MAVlist)
                {
                    if (mav == Leader) continue;
                    Vector3 off = getOffsets(mav);
                    double hdg = -Leader.cs.yaw * (Math.PI / 180.0);
                    Vector3 targetUTM = new Vector3(
                        leaderUTM.x + off.x * (float)Math.Cos(hdg) - off.y * (float)Math.Sin(hdg),
                        leaderUTM.y + off.x * (float)Math.Sin(hdg) + off.y * (float)Math.Cos(hdg),
                        (float)masterpos.Alt + off.z
                    );
                    double[] inv = trans.MathTransform.Inverse().Transform(new[] { targetUTM.x, targetUTM.y });
                    var targetGeo = new PointLatLngAlt(inv[1], inv[0], targetUTM.z, "");

                    if (mav.cs.firmware == Firmwares.ArduPlane)
                    {
                        DateTime now = DateTime.UtcNow;
                        float dt = (float)(now - lastUpdateTime).TotalSeconds;

                        // 1) Analytic ff accel
                        Vector3 aL = new Vector3(Leader.cs.ax, Leader.cs.ay, Leader.cs.az);
                        float yawR = payloadController.EstimateYawRate(Leader);
                        float yawA = prevYawRate.TryGetValue(mav, out var pr) ? (yawR - pr) / dt : 0f;
                        prevYawRate[mav] = yawR;
                        Vector3 a_ff = aL
                            + new Vector3(-yawR * yawR * off.x, -yawR * yawR * off.y, 0f)
                            + new Vector3(-off.y * yawA, off.x * yawA, 0f);

                        // 2) Feedback accel
                        double[] cXY = trans.MathTransform.Transform(new[] { mav.cs.lng, mav.cs.lat });
                        Vector3 curUTM = new Vector3((float)cXY[0], (float)cXY[1], mav.cs.alt);
                        Vector3 e_pos = targetUTM - curUTM;
                        Vector3 curV = new Vector3(mav.cs.vx, mav.cs.vy, mav.cs.vz);
                        Vector3 v_ff = new Vector3(Leader.cs.vx, Leader.cs.vy, Leader.cs.vz)
                            + new Vector3(-yawR * off.y, yawR * off.x, 0f);
                        Vector3 e_vel = v_ff - curV;
                        Vector3 a_fb = e_pos * Kp + e_vel * Kv;

                        // 3) Tension comp w/ null-space
                        Vector3 comp = payloadController.CompensateRigidBodyDynamics(Leader, mav, offsets);
                        float alpha = dt / (compTau + dt);
                        Vector3 prevC = compFiltered.ContainsKey(mav) ? compFiltered[mav] : Vector3.Zero;
                        Vector3 smooth = prevC + (comp - prevC) * alpha;
                        compFiltered[mav] = smooth;

                        // 4) Total accel
                        Vector3 u = a_ff + a_fb + smooth * payloadGain;

                        // 5) Consensus term
                        float Kc = 1.0f;
                        Vector3 consensus = Vector3.Zero;
                        foreach (var nei in GetNeighbors(mav))
                        {
                            double[] nXY = trans.MathTransform.Transform(new[] { nei.cs.lng, nei.cs.lat });
                            Vector3 nUTM = new Vector3((float)nXY[0], (float)nXY[1], nei.cs.alt);
                            consensus += (nUTM - curUTM);
                        }
                        u += consensus * Kc;

                        // 6) Attitude + thrust
                        double phi = Math.Asin(Math.Max(-1, Math.Min(1, u.y / 9.81))) * (180.0 / Math.PI);
                        double tht = Math.Asin(Math.Max(-1, Math.Min(1, -u.x / 9.81))) * (180.0 / Math.PI);
                        float thrust = (float)Math.Max(0.1, Math.Min(1, (u.z + 9.81) / 9.81));

                        // 7) FBAC adaptation
                        float tensionErr = (float)(e_pos.x * smooth.x + e_pos.y * smooth.y + e_pos.z * smooth.z);
                        float posErrSq = (float)(e_pos.x * e_pos.x + e_pos.y * e_pos.y + e_pos.z * e_pos.z);
                        float velErrSq = (float)(e_vel.x * e_vel.x + e_vel.y * e_vel.y + e_vel.z * e_vel.z);
                        payloadGain = Math.Max(0f, payloadGain + gammaPayload * tensionErr * dt);
                        Kp = Math.Max(0.1f, Kp + gammaKp * posErrSq * dt);
                        Kv = Math.Max(0.1f, Kv + gammaKv * velErrSq * dt);

                        // 8) Yaw ff + fb
                        double psi_ff = Leader.cs.yaw + yawR * dt;
                        double psi_err = wrap_180(psi_ff - mav.cs.yaw);
                        var quat = Quaternion.from_euler312((float)(phi * Math.PI / 180.0), (float)(tht * Math.PI / 180.0), (float)(psi_err * Math.PI / 180.0));
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

        // Placeholder for neighbor lookup; implement based on offsets or topology
        private IEnumerable<MAVState> GetNeighbors(MAVState mav)
        {
            foreach (var kv in offsets)
                if (kv.Key != mav)
                    yield return kv.Key;
        }
    }

    // PID Controller Implementation
    internal class PID
    {
        private float dt, input, deriv, integrator, imax, filtHz;
        private readonly float kp, ki, kd;
        private bool resetFilter = true;

        public PID(float p, float i, float d, float im, float fHz, float dt0, float ff)
        {
            kp = p; ki = i; kd = d; imax = Math.Abs(im);
            filtHz = Math.Max(fHz, 0.01f); dt = dt0;
        }
        public void set_input_filter_all(float inVal)
        {
            if (float.IsInfinity(inVal)) return;
            if (resetFilter) { input = inVal; deriv = 0; resetFilter = false; }
            float alpha = dt / (dt + 1f / (2f * (float)Math.PI * filtHz));
            float delta = alpha * (inVal - input);
            input += delta;
            deriv = dt > 0 ? delta / dt : 0;
        }
        public float get_pid()
        {
            float P = kp * input;
            integrator = Math.Max(-imax, Math.Min(imax, integrator + ki * input * dt));
            float D = kd * deriv;
            return P + integrator + D;
        }
    }

    /// <summary>Vector utilities</summary>
    internal static class VectorUtils
    {
        public static Vector3 NormalizeVector(Vector3 v)
        {
            double len = Math.Sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
            return len == 0 ? Vector3.Zero : new Vector3((float)(v.x / len), (float)(v.y / len), (float)(v.z / len));
        }
    }

    /// <summary>Payload wrench solver with null-space tension balancing</summary>
    internal class LoadAttitudeController
    {
        public double PayloadMass { get; set; } = 1.0;
        private static readonly Matrix<double> JL = Matrix<double>.Build.DenseOfArray(new double[,] { { 0.021, 0, 0 }, { 0, 0.0187, 0 }, { 0, 0, 0.0397 } });
        private static readonly Dictionary<int, Tuple<float, float>> YawHistory = new Dictionary<int, Tuple<float, float>>();

        public float EstimateYawRate(MAVState leader)
        {
            int id = leader.sysid;
            float cy = leader.cs.yaw;
            float ct = (float)(DateTime.UtcNow - new DateTime(1970, 1, 1)).TotalSeconds;
            if (YawHistory.ContainsKey(id))
            {
                var (py, pt) = YawHistory[id];
                float dy = (float)Wrap(cy - py);
                float dt = ct - pt;
                YawHistory[id] = Tuple.Create(cy, ct);
                return dt > 1e-3f ? dy / dt : 0f;
            }
            YawHistory[id] = Tuple.Create(cy, ct);
            return 0f;
        }

        public Vector3 CompensateRigidBodyDynamics(MAVState leader, MAVState follower, Dictionary<MAVState, Vector3> offsets)
        {
            if (!offsets.ContainsKey(follower)) return Vector3.Zero;
            var pts = new List<Vector3>(offsets.Values);
            if (pts.Count < 3) return Vector3.Zero;
            var rOff = offsets[follower];
            float yawRate = EstimateYawRate(leader);
            float compTc = Math.Abs(yawRate) > 10 ? 0.05f : 0.2f;
            float dt = 0.05f;
            float alpha = dt / (compTc + dt);
            float m = (float)PayloadMass;
            var accel = Vector<double>.Build.DenseOfArray(new double[] { leader.cs.ax * m, leader.cs.ay * m, (leader.cs.az + 9.81f) * m });
            var grav = Vector<double>.Build.Dense(new[] { 0.0, 0.0, -9.81 }) * m;
            var R = BuildRotationMatrix(leader.cs.roll, leader.cs.pitch, leader.cs.yaw);
            var Wf = -R.Transpose() * (accel + grav);
            var omega = Vector<double>.Build.Dense(3);
            var omegaDot = Vector<double>.Build.Dense(3);
            var Wm = -(JL * omegaDot + Cross(omega, JL * omega));
            var W = Vector<double>.Build.Dense(6);
            W.SetSubVector(0, 3, Wf);
            W.SetSubVector(3, 3, Wm);
            var Phi = Matrix<double>.Build.Dense(6, pts.Count);
            for (int i = 0; i < pts.Count; i++)
            {
                var r = pts[i];
                var qi = VectorUtils.NormalizeVector(r);
                Phi[0, i] = qi.x; Phi[1, i] = qi.y; Phi[2, i] = qi.z;
                Phi[3, i] = r.y * qi.z - r.z * qi.y;
                Phi[4, i] = r.z * qi.x - r.x * qi.z;
                Phi[5, i] = r.x * qi.y - r.y * qi.x;
            }
            var pinv = Phi.PseudoInverse();
            var Tpart = pinv * W;
            var N = Matrix<double>.Build.DenseIdentity(pts.Count) - pinv * Phi;
            double avgT = Tpart.Sum() / pts.Count;
            var equal = Vector<double>.Build.Dense(pts.Count, avgT);
            var Tnull = N * equal;
            var Tfinal = Tpart + Tnull;
            int idx = new List<MAVState>(offsets.Keys).IndexOf(follower);
            if (idx >= 0 && idx < Tfinal.Count)
            {
                var qi = VectorUtils.NormalizeVector(rOff);
                return qi * (float)Math.Max(0, Tfinal[idx]) * alpha;
            }
            return Vector3.Zero;
        }

        private static double Wrap(double a) => a > 180 ? a - 360 : a < -180 ? a + 360 : a;
        private static Matrix<double> BuildRotationMatrix(float roll, float pitch, float yaw)
        {
            double r = roll * (Math.PI / 180.0), p = pitch * (Math.PI / 180.0), y = yaw * (Math.PI / 180.0);
            double cr = Math.Cos(r), sr = Math.Sin(r);
            double cp = Math.Cos(p), sp = Math.Sin(p);
            double cy = Math.Cos(y), sy = Math.Sin(y);
            return Matrix<double>.Build.DenseOfArray(new double[,] {
                { cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr },
                { sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr },
                { -sp,   cp*sr,            cp*cr           }
            });
        }
        private static Vector<double> Cross(Vector<double> a, Vector<double> b) =>
            Vector<double>.Build.DenseOfArray(new double[] {
                a[1]*b[2] - a[2]*b[1],
                a[2]*b[0] - a[0]*b[2],
                a[0]*b[1] - a[1]*b[0]
            });
    }
}
