using System;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using MissionPlanner.ArduPilot;
using MissionPlanner.Utilities;
using ProjNet.CoordinateSystems;
using ProjNet.CoordinateSystems.Transformations;
using GeoAPI.CoordinateSystems;
using GeoAPI.CoordinateSystems.Transformations;
using Vector3 = MissionPlanner.Utilities.Vector3;

namespace MissionPlanner.Swarm
{
    /// <summary>Constants for formation control.</summary>
    internal static class FormationConstants
    {
        public const double Gravity = 9.81;
        public const double PayloadMass = 1.0; // fixed mass (kg)
        public static readonly Matrix<double> PayloadInertia = Matrix<double>.Build.DenseOfArray(new double[,]
        {
            { 0.021, 0.0,   0.0 },
            { 0.0,   0.0187,0.0 },
            { 0.0,   0.0,   0.0397 }
        });
        public const float YawRateDt = 0.01f;
        public const double AdaptationGain = 0.5;
        public const double MinCompGain = 0.1;
        public const double MaxCompGain = 2.0;
    }

    /// <summary>Math utilities.</summary>
    internal static class MathUtils
    {
        public const double Deg2Rad = Math.PI / 180.0;
        public const double Rad2Deg = 180.0 / Math.PI;
        public static double Constrain(double val, double min, double max) => val < min ? min : (val > max ? max : val);
        public static double WrapDegrees(double ang)
        {
            if (ang > 180) ang -= 360;
            else if (ang < -180) ang += 360;
            return ang;
        }
    }

    /// <summary>Compute mapping Φ, wrench W, and tensions with adaptation.</summary>
    internal class LoadAttitudeController
    {
        private readonly Dictionary<int, float> lastYaw = new Dictionary<int, float>();

        // Adaptive compensation gain
        private double compGainHat = 1.0;
        public double CompGainHat => compGainHat;

        public Matrix<double> ComputeMappingMatrix(Dictionary<MAVState, Vector3> offsets)
        {
            int n = offsets.Count;
            var Phi = Matrix<double>.Build.Dense(6, 3 * n);
            var pts = new List<Vector3>(offsets.Values);
            for (int i = 0; i < n; i++)
            {
                var r = pts[i];
                var q = VectorUtils.Normalize(r);
                // Force rows
                for (int ax = 0; ax < 3; ax++)
                    Phi[ax, 3 * i + ax] = q[ax];
                // Moment rows: hat(r) * q
                var hat = Matrix<double>.Build.DenseOfArray(new double[,]
                {
                    {0, -r.z, r.y}, {r.z, 0, -r.x}, {-r.y, r.x, 0}
                });
                var mvec = hat * Vector<double>.Build.Dense(new[] { q.x, q.y, q.z });
                for (int ax = 0; ax < 3; ax++)
                    for (int col = 0; col < 3; col++)
                        Phi[3 + ax, 3 * i + col] = mvec[ax];
            }
            return Phi;
        }

        public Vector<double> ComputePayloadWrench(MAVState leader)
        {
            // Force component
            double mL = FormationConstants.PayloadMass;
            var aL = Vector<double>.Build.Dense(new[]
            {
                leader.cs.ax * mL,
                leader.cs.ay * mL,
                (leader.cs.az + (float)FormationConstants.Gravity) * mL
            });
            var R = BuildRotation(leader.cs.roll, leader.cs.pitch, leader.cs.yaw);
            var Wf = -R.Transpose() * aL;

            // Rotational omitted: zero moment
            var Wm = Vector<double>.Build.Dense(3);

            var W = Vector<double>.Build.Dense(6);
            W.SetSubVector(0, 3, Wf);
            W.SetSubVector(3, 3, Wm);
            return W;
        }

        public Vector<double> ComputeTensions(Matrix<double> Phi, Vector<double> W)
            => Phi.PseudoInverse() * W;

        public void UpdateCompGain(Vector3 rawComp, Vector3 error, float dt)
        {
            double dot = rawComp.x * error.x + rawComp.y * error.y + rawComp.z * error.z;
            compGainHat += FormationConstants.AdaptationGain * dot * dt;
            compGainHat = MathUtils.Constrain(compGainHat, FormationConstants.MinCompGain, FormationConstants.MaxCompGain);
        }

        private Matrix<double> BuildRotation(float roll, float pitch, float yaw)
        {
            double r = roll * MathUtils.Deg2Rad;
            double p = pitch * MathUtils.Deg2Rad;
            double y = yaw * MathUtils.Deg2Rad;
            double cr = Math.Cos(r), sr = Math.Sin(r);
            double cp = Math.Cos(p), sp = Math.Sin(p);
            double cy = Math.Cos(y), sy = Math.Sin(y);
            return Matrix<double>.Build.DenseOfArray(new double[,]
            {
                { cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr },
                { sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr },
                {   -sp,            cp*sr,            cp*cr }
            });
        }
    }

    internal static class VectorExtensions
    {
        public static Vector<double> Cross(this Vector<double> a, Vector<double> b)
            => Vector<double>.Build.Dense(new[]
            {
                a[1]*b[2] - a[2]*b[1],
                a[2]*b[0] - a[0]*b[2],
                a[0]*b[1] - a[1]*b[0]
            });
    }

    internal static class VectorUtils
    {
        public static Vector3 Normalize(Vector3 v)
        {
            double len = Math.Sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
            return len < 1e-6 ? Vector3.Zero : new Vector3((float)(v.x / len), (float)(v.y / len), (float)(v.z / len));
        }
    }

    internal class Formation : Swarm
    {
        private readonly Dictionary<MAVState, Vector3> offsets = new Dictionary<MAVState, Vector3>();
        private readonly Dictionary<MAVState, Vector3> desiredAccel = new Dictionary<MAVState, Vector3>();
        private readonly LoadAttitudeController ctrl = new LoadAttitudeController();
        private PointLatLngAlt payloadPose = new PointLatLngAlt();

        public void SetDesiredAcceleration(MAVState mav, Vector3 accel) => desiredAccel[mav] = accel;
        public void SetOffsets(MAVState mav, double x, double y, double z) => offsets[mav] = new Vector3((float)x, (float)y, (float)z);

        public override void Update()
        {
            var mavs = new List<MAVState>();
            foreach (var port in MainV2.Comports) mavs.AddRange(port.MAVlist);
            mavs.RemoveAll(m => m.cs.lat == 0 && m.cs.lng == 0);
            if (mavs.Count == 0) return;
            double sumLat = 0, sumLng = 0, sumAlt = 0;
            mavs.ForEach(m => { sumLat += m.cs.lat; sumLng += m.cs.lng; sumAlt += m.cs.alt; });
            payloadPose.Lat = sumLat / mavs.Count;
            payloadPose.Lng = sumLng / mavs.Count;
            payloadPose.Alt = sumAlt / mavs.Count;
            if (Leader == null || Leader.cs.lat == 0) Leader = mavs[0];
        }

        public override void SendCommand()
        {
            if (payloadPose.Lat == 0 && payloadPose.Lng == 0) return;
            const float dt = 0.02f;
            foreach (var port in MainV2.Comports)
                foreach (var mav in port.MAVlist)
                {
                    if (mav == Leader) continue;
                    var u_ff = desiredAccel.ContainsKey(mav) ? desiredAccel[mav] : Vector3.Zero;
                    var Phi = ctrl.ComputeMappingMatrix(offsets);
                    var W = ctrl.ComputePayloadWrench(Leader);
                    var T = ctrl.ComputeTensions(Phi, W);
                    int idx = new List<MAVState>(offsets.Keys).IndexOf(mav);
                    float Ti = idx >= 0 ? (float)MathUtils.Constrain((float)T[idx], 0, float.MaxValue) : 0f;
                    var q_i = offsets.ContainsKey(mav) ? VectorUtils.Normalize(offsets[mav]) : Vector3.Zero;
                    var rawComp = q_i * Ti;
                    var actual = new Vector3(mav.cs.ax, mav.cs.ay, mav.cs.az);
                    var comp = rawComp * (float)ctrl.CompGainHat;
                    var error = actual - (u_ff + comp);
                    ctrl.UpdateCompGain(rawComp, error, dt);
                    var cmdAccel = u_ff + comp;
                    SendPositionOrAttitude(port, mav, cmdAccel);
                }
        }

        private void SendPositionOrAttitude(dynamic port, MAVState mav, Vector3 accel)
        {
            var bearing = new PointLatLngAlt(payloadPose).GetBearing(mav.cs.Location);
            double yawErr = bearing - mav.cs.yaw;
            double yawRad = MathUtils.WrapDegrees(yawErr) * MathUtils.Deg2Rad;
            if (mav.cs.firmware == Firmwares.ArduPlane)
            {
                float roll = (float)Math.Atan2(accel.x, accel.z + (float)FormationConstants.Gravity);
                float pitch = (float)Math.Atan2(-accel.y, accel.z + (float)FormationConstants.Gravity);
                var q = Quaternion.from_euler312(roll, pitch, (float)yawRad);
                port.sendPacket(new MAVLink.mavlink_set_attitude_target_t
                {
                    target_system = mav.sysid,
                    target_component = mav.compid,
                    type_mask = 0b10000101,
                    q = new float[] { (float)q.q1, (float)q.q2, (float)q.q3, (float)q.q4 }
                }, mav.sysid, mav.compid);
            }
            else
            {
                port.setPositionTargetGlobalInt(
                    mav.sysid, mav.compid, true, true, false, false,
                    MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT_INT,
                    payloadPose.Lat, payloadPose.Lng, payloadPose.Alt,
                    accel.x, accel.y, accel.z, 0, 0);
            }
        }
    }
}
