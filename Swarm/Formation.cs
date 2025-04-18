using System;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using ProjNet.CoordinateSystems;
using ProjNet.CoordinateSystems.Transformations;
using GeoAPI.CoordinateSystems;
using GeoAPI.CoordinateSystems.Transformations;
using MissionPlanner.ArduPilot;
using MissionPlanner.Utilities;

namespace MissionPlanner.Swarm
{
    // --- Geodetic Transform Helpers ---
    internal static class GeoHelper
    {
        private static readonly IGeographicCoordinateSystem _wgs84 = GeographicCoordinateSystem.WGS84;
        private static readonly CoordinateTransformationFactory _ctfac = new CoordinateTransformationFactory();

        public static (double x, double y, int zone, bool north) LatLonToUtm(double lat, double lon)
        {
            int zone = (int)((lon + 180.0) / 6.0) + 1;
            bool northern = lat >= 0;
            var utm = ProjectedCoordinateSystem.WGS84_UTM(zone, northern);
            var trans = _ctfac.CreateFromCoordinateSystems(_wgs84, utm);
            double[] xy = trans.MathTransform.Transform(new double[] { lon, lat });
            return (xy[0], xy[1], zone, northern);
        }

        public static (double lat, double lon) UtmToLatLon(double x, double y, int zone, bool northern)
        {
            var utm = ProjectedCoordinateSystem.WGS84_UTM(zone, northern);
            var trans = _ctfac.CreateFromCoordinateSystems(utm, _wgs84);
            double[] ll = trans.MathTransform.Transform(new double[] { x, y });
            return (ll[1], ll[0]);
        }
    }

    // --- Basic 3D Vector and Wrapper ---
    public class Vector3
    {
        public float X { get; set; }
        public float Y { get; set; }
        public float Z { get; set; }
        public Vector3(float x, float y, float z) { X = x; Y = y; Z = z; }
        public static Vector3 Zero => new Vector3(0f, 0f, 0f);
        public Vector3 Multiply(float s) => new Vector3(X * s, Y * s, Z * s);
    }

    public class Vector3Wrapper
    {
        public Vector3 V { get; private set; }
        public Vector3Wrapper(float x, float y, float z) { V = new Vector3(x, y, z); }
        public static Vector3Wrapper Add(Vector3Wrapper a, Vector3Wrapper b) =>
            new Vector3Wrapper(a.V.X + b.V.X, a.V.Y + b.V.Y, a.V.Z + b.V.Z);
        public static Vector3Wrapper Multiply(Vector3Wrapper v, float s) =>
            new Vector3Wrapper(v.V.X * s, v.V.Y * s, v.V.Z * s);
        public static Vector3Wrapper Zero => new Vector3Wrapper(0f, 0f, 0f);
    }

    // --- PID with Sigma Modification (P, I, D) ---
    internal class PIDController
    {
        public float Kp, Ki, Kd;
        public float lastError;
        private float integrator;

        public PIDController(float kp, float ki, float kd)
        {
            Kp = kp; Ki = ki; Kd = kd;
            integrator = 0f; lastError = 0f;
        }

        public float Step(float error, float dError, float dt)
        {
            integrator += error * dt;
            return Kp * error + Ki * integrator + Kd * dError;
        }

        public void ClampLower(float kpMin, float kiMin, float kdMin)
        {
            if (Kp < kpMin) Kp = kpMin;
            if (Ki < kiMin) Ki = kiMin;
            if (Kd < kdMin) Kd = kdMin;
        }
    }

    // --- Utility Math ---
    internal static class MathHelper
    {
        public const double Deg2Rad = Math.PI / 180.0;
        public const double Rad2Deg = 180.0 / Math.PI;
        public static float Constrain(float v, float min, float max) => v < min ? min : v > max ? max : v;
    }

    internal static class VectorUtils
    {
        public static Vector3Wrapper Normalize(Vector3Wrapper w)
        {
            var v = w.V;
            double len = Math.Sqrt((double)v.X * v.X + (double)v.Y * v.Y + (double)v.Z * v.Z);
            if (len <= 0) return Vector3Wrapper.Zero;
            float inv = 1f / (float)len;
            return new Vector3Wrapper(v.X * inv, v.Y * inv, v.Z * inv);
        }
        public static float Length(Vector3 v) => (float)Math.Sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z);
        public static float LengthSquared(Vector3 v) => v.X * v.X + v.Y * v.Y + v.Z * v.Z;
    }

    // --- Swarm Constants ---
    internal static class SwarmConstants
    {
        public const float PayloadMass = 1.0f;   // Payload mass (kg)
        public const float DroneMass = 1.5f;     // Individual drone mass (kg)
        public const float CableLength = 2.0f;   // Cable length (m)
    }

    // --- Load Attitude Controller ---
    internal class LoadAttitudeController
    {
        public float PayloadMass = SwarmConstants.PayloadMass;
        private static readonly Matrix<double> Iinertia =
            Matrix<double>.Build.DenseOfArray(new double[,] {
                { 0.021, 0.0,   0.0   },
                { 0.0,   0.0187,0.0   },
                { 0.0,   0.0,   0.0397}
            });

        public Vector3Wrapper Compensate(
            MAVState leader,
            MAVState follower,
            Dictionary<MAVState, Vector3Wrapper> offsets,
            out Matrix<double> nullspaceBasis)
        {
            float m = PayloadMass;
            var accel = Vector<double>.Build.Dense(new double[] {
                leader.cs.ax * m,
                leader.cs.ay * m,
                (leader.cs.az + 9.81f) * m
            });
            var grav = Vector<double>.Build.Dense(new double[] { 0.0, 0.0, -9.81 }) * m;
            var R = BuildRot(leader.cs.roll, leader.cs.pitch, leader.cs.yaw);
            var Wf = -R.Transpose() * (accel + grav);
            var omega = Vector<double>.Build.Dense(3);
            var Wm = -(Iinertia * Vector<double>.Build.Dense(3) + Cross(omega, Iinertia * omega));
            var W = Vector<double>.Build.Dense(6);
            W.SetSubVector(0, 3, Wf);
            W.SetSubVector(3, 3, Wm);

            var pts = new List<Vector3Wrapper>(offsets.Values);
            int n = pts.Count;
            var Phi = Matrix<double>.Build.Dense(6, n);
            for (int i = 0; i < n; i++)
            {
                var orig = pts[i].V;
                float curLen = VectorUtils.Length(orig);
                var rvec = curLen > 0
                    ? new Vector3(
                          orig.X * (SwarmConstants.CableLength / curLen),
                          orig.Y * (SwarmConstants.CableLength / curLen),
                          orig.Z * (SwarmConstants.CableLength / curLen)
                      )
                    : orig;
                var qi3 = VectorUtils.Normalize(new Vector3Wrapper(rvec.X, rvec.Y, rvec.Z)).V;
                Phi[0, i] = qi3.X;  Phi[1, i] = qi3.Y;  Phi[2, i] = qi3.Z;
                Phi[3, i] = rvec.Y * qi3.Z - rvec.Z * qi3.Y;
                Phi[4, i] = rvec.Z * qi3.X - rvec.X * qi3.Z;
                Phi[5, i] = rvec.X * qi3.Y - rvec.Y * qi3.X;
            }

            var pinv = Phi.PseudoInverse();
            var svd = Phi.Svd(true);
            nullspaceBasis = svd.VT.Transpose().SubMatrix(0, n, 6, n - 6);
            var Tp = pinv * W;
            var lambda = Vector<double>.Build.Dense(n - 6);
            var T = Tp + nullspaceBasis * lambda;

            int idx = new List<MAVState>(offsets.Keys).IndexOf(follower);
            if (idx < 0 || idx >= T.Count)
                return Vector3Wrapper.Zero;

            var soff = offsets[follower].V;
            var so3 = VectorUtils.Normalize(new Vector3Wrapper(soff.X, soff.Y, soff.Z)).V;
            float alpha = 0.2f;
            float tension = (float)Math.Max(0, T[idx]);
            return new Vector3Wrapper(
                so3.X * tension * alpha,
                so3.Y * tension * alpha,
                so3.Z * tension * alpha
            );
        }

        private Matrix<double> BuildRot(float roll, float pitch, float yaw)
        {
            double r = roll * MathHelper.Deg2Rad;
            double p = pitch * MathHelper.Deg2Rad;
            double y = yaw * MathHelper.Deg2Rad;
            double cr = Math.Cos(r), sr = Math.Sin(r);
            double cp = Math.Cos(p), sp = Math.Sin(p);
            double cy = Math.Cos(y), sy = Math.Sin(y);
            return Matrix<double>.Build.DenseOfArray(new double[,] {
                { cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr },
                { sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr },
                { -sp,     cp * sr,                cp * cr           }
            });
        }

        private Vector<double> Cross(Vector<double> a, Vector<double> b)
        {
            return Vector<double>.Build.Dense(new double[] {
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]
            });
        }
    }

    // --- Formation Controller ---
    internal class Formation : Swarm
    {
        private Dictionary<MAVState, Vector3Wrapper> offsets = new Dictionary<MAVState, Vector3Wrapper>();
        private Dictionary<MAVState, (PIDController roll, PIDController pitch)> pids = new Dictionary<MAVState, (PIDController, PIDController)>();
        private LoadAttitudeController loadCtrl = new LoadAttitudeController();
        private PointLatLngAlt masterpos = new PointLatLngAlt();
        private DateTime lastTime = DateTime.UtcNow;

        // --- Adaptive control constants ---
        private const float sigma = 0.1f;                          // sigma-modification
        private const float Kp0 = 1f, Ki0 = 0.1f, Kd0 = 0.01f;     // position gain
        private const float Kv0 = 1f;                              // velocity gain
        private const float Gi = 0.1f, Gd = 0.01f;                 // adaptation rate

                public void SetOffsets(MAVState m, float x, float y, float z)
            => offsets[m] = new Vector3Wrapper(x, y, z);

        // Getter for offsets
        public Vector3Wrapper GetOffsets(MAVState m)
    => offsets.TryGetValue(m, out var w) ? w : Vector3Wrapper.Zero;

        // Override Swarm.Update() refresh leader position and timestamp
        public override void Update()
        {
            if (MainV2.comPort.MAV.cs.lat == 0 || MainV2.comPort.MAV.cs.lng == 0)
                return;
            if (Leader == null)
                Leader = MainV2.comPort.MAV;
            masterpos = new PointLatLngAlt(Leader.cs.lat, Leader.cs.lng, Leader.cs.alt, "");
            lastTime = DateTime.UtcNow;
        }

        public override void SendCommand()
        {
            if (masterpos.Lat == 0 || masterpos.Lng == 0) return;

            var (lx, ly, zone, north) = GeoHelper.LatLonToUtm(masterpos.Lat, masterpos.Lng);
            float heading = (float)(-Leader.cs.yaw * MathHelper.Deg2Rad);

            foreach (var port in MainV2.Comports)
            foreach (var mav in port.MAVlist)
            {
                if (mav == Leader) continue;

                var comp = loadCtrl.Compensate(Leader, mav, offsets, out var N);

                var off = offsets[mav].V;
                double x2 = lx + off.X * Math.Cos(heading) - off.Y * Math.Sin(heading);
                double y2 = ly + off.X * Math.Sin(heading) + off.Y * Math.Cos(heading);
                var (tlat, tlon) = GeoHelper.UtmToLatLon(x2, y2, zone, north);
                double talt = masterpos.Alt + off.Z;

                if (!pids.ContainsKey(mav))
                    pids[mav] = (new PIDController(Kp0, Ki0, Kd0), new PIDController(Kv0, Ki0, Kd0));
                var (pidP, pidV) = pids[mav];

                var curPos = new Vector3Wrapper((float)mav.cs.lat, (float)mav.cs.lng, (float)mav.cs.alt);
                var tarPos = new Vector3Wrapper((float)tlat, (float)tlon, (float)talt);
                var posErr = Vector3Wrapper.Add(tarPos, Vector3Wrapper.Multiply(curPos, -1f));

                var curVel = new Vector3Wrapper((float)mav.cs.vx, (float)mav.cs.vy, (float)mav.cs.vz);
                var velErr = Vector3Wrapper.Add(Vector3Wrapper.Zero, curVel);

                float dt = (float)(DateTime.UtcNow - lastTime).TotalSeconds;

                pidP.Kp += sigma * (pidP.Kp - Kp0) + Gi * VectorUtils.LengthSquared(posErr.V);
                pidP.Ki += sigma * (pidP.Ki - Ki0) + Gi * VectorUtils.LengthSquared(posErr.V);
                pidP.Kd += sigma * (pidP.Kd - Kd0) + Gd * VectorUtils.LengthSquared(velErr.V);
                pidV.Kp += sigma * (pidV.Kp - Kv0) + Gi * VectorUtils.LengthSquared(velErr.V);
                pidV.Ki += sigma * (pidV.Ki - Ki0) + Gi * VectorUtils.LengthSquared(velErr.V);
                pidV.Kd += sigma * (pidV.Kd - Kd0) + Gd * VectorUtils.LengthSquared(velErr.V);

                pidP.ClampLower(Kp0, Ki0, Kd0);
                pidV.ClampLower(Kv0, Ki0, Kd0);

                float dPos = (posErr.V.X - pidP.lastError) / dt;
                float outP = pidP.Step(posErr.V.X, dPos, dt);
                pidP.lastError = posErr.V.X;

                float dVel = (velErr.V.X - pidV.lastError) / dt;
                float outV = pidV.Step(velErr.V.X, dVel, dt);
                pidV.lastError = velErr.V.X;

                float cmd = outP + outV + comp.V.X;

                // position+velocity target (15-arg overload)
                var leaderVel = new Vector3Wrapper((float)Leader.cs.vx, (float)Leader.cs.vy, (float)Leader.cs.vz);
                port.setPositionTargetGlobalInt(
                    mav.sysid,
                    mav.compid,
                    true,   // use position
                    true,   // use velocity
                    false,  // use acceleration
                    false,  // is force
                    MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT_INT,
                    (float)tlat,  // latitude (deg)
                    (float)tlon,  // longitude (deg)
                    (float)talt,  // altitude (m)
                    leaderVel.V.X, // vx (m/s)
                    leaderVel.V.Y, // vy (m/s)
                    leaderVel.V.Z, // vz (m/s)
                    0f,           // afx (ignored)
                    0f            // afy (ignored)
                );
            }
        }
    }
}
