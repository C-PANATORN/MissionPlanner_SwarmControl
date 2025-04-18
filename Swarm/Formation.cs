// This file is refactored for C# 7.3 compatibility with proper aliasing
using System;
using System.Collections.Generic;
using System.Numerics;
using MathNet.Numerics.LinearAlgebra;
using ProjNet.CoordinateSystems;
using ProjNet.CoordinateSystems.Transformations;
using GeoAPI.CoordinateSystems;
using GeoAPI.CoordinateSystems.Transformations;
using MissionPlanner.ArduPilot;
using MissionPlanner.Utilities;

// Type aliases to avoid ambiguity
using Vec3 = System.Numerics.Vector3;
using MathVec = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MathMat = MathNet.Numerics.LinearAlgebra.Matrix<double>;

namespace MissionPlanner.Swarm
{
    // --- Geodetic Transform Helpers ---
    internal static class GeoHelper
    {
        private static readonly IGeographicCoordinateSystem Wgs84 = GeographicCoordinateSystem.WGS84;
        private static readonly CoordinateTransformationFactory Ctfac = new CoordinateTransformationFactory();

        public static (double X, double Y, int Zone, bool Northern) LatLonToUtm(double lat, double lon)
        {
            int zone = (int)((lon + 180.0) / 6.0) + 1;
            bool northern = lat >= 0;
            var utm = ProjectedCoordinateSystem.WGS84_UTM(zone, northern);
            var trans = Ctfac.CreateFromCoordinateSystems(Wgs84, utm);
            double[] xy = trans.MathTransform.Transform(new double[] { lon, lat });
            return (xy[0], xy[1], zone, northern);
        }

        public static (double Lat, double Lon) UtmToLatLon(double x, double y, int zone, bool northern)
        {
            var utm = ProjectedCoordinateSystem.WGS84_UTM(zone, northern);
            var trans = Ctfac.CreateFromCoordinateSystems(utm, Wgs84);
            double[] ll = trans.MathTransform.Transform(new double[] { x, y });
            return (ll[1], ll[0]);
        }
    }

    // --- PID with sigma-mod adaptation ---
    internal class PIDController
    {
        public float Kp, Ki, Kd;
        public float LastError;
        private float integrator;

        public PIDController(float kp, float ki, float kd)
        {
            Kp = kp;
            Ki = ki;
            Kd = kd;
            LastError = 0f;
            integrator = 0f;
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

    // --- Swarm Constants ---
    internal static class SwarmConstants
    {
        public const float PayloadMass = 1.0f;
        public const float DroneMass = 1.5f;
        public const float CableLength = 2.0f;
        public const double Deg2Rad = Math.PI / 180.0;
    }

    // --- Load Attitude Controller ---
    internal class LoadAttitudeController
    {
        private static readonly MathMat Inertia = MathMat.Build.DenseOfArray(
            new double[,] { { 0.021, 0,     0     },
                            { 0,     0.0187,0     },
                            { 0,     0,     0.0397} }
        );

        public Vec3 Compensate(
            MAVState leader,
            MAVState follower,
            IDictionary<MAVState, Vec3> offsets,
            out MathMat nullspace)
        {
            float m = SwarmConstants.PayloadMass;
            MathVec accel = MathVec.Build.Dense(new double[] {
                leader.cs.ax * m,
                leader.cs.ay * m,
                (leader.cs.az + 9.81f) * m
            });
            MathVec grav = MathVec.Build.Dense(new double[] { 0.0, 0.0, -9.81 }) * m;
            MathMat R = BuildRotation(leader.cs.roll, leader.cs.pitch, leader.cs.yaw);

            MathVec Wf = -R.Transpose() * (accel + grav);
            MathVec omega = MathVec.Build.Dense(3);
            MathVec Wm = -(Inertia * MathVec.Build.Dense(3) + Cross(omega, Inertia * omega));

            MathVec W = MathVec.Build.Dense(6);
            W.SetSubVector(0, 3, Wf);
            W.SetSubVector(3, 3, Wm);

            var pts = new List<Vec3>(offsets.Values);
            int n = pts.Count;
            MathMat Phi = MathMat.Build.Dense(6, n);
            for (int i = 0; i < n; i++)
            {
                Vec3 orig = pts[i];
                float curLen = orig.Length();
                Vec3 rvec = curLen > 0
                    ? orig * (SwarmConstants.CableLength / curLen)
                    : orig;
                Vec3 qi = Vec3.Normalize(rvec);

                Phi[0, i] = qi.X;
                Phi[1, i] = qi.Y;
                Phi[2, i] = qi.Z;
                Phi[3, i] = rvec.Y * qi.Z - rvec.Z * qi.Y;
                Phi[4, i] = rvec.Z * qi.X - rvec.X * qi.Z;
                Phi[5, i] = rvec.X * qi.Y - rvec.Y * qi.X;
            }

            MathMat pinv = Phi.PseudoInverse();
            var svd = Phi.Svd(true);
            nullspace = svd.VT.Transpose().SubMatrix(0, n, 6, n - 6);

            MathVec Tp = pinv * W;
            MathVec lambda = MathVec.Build.Dense(n - 6);
            MathVec T = Tp + nullspace * lambda;

            int idx = new List<MAVState>(offsets.Keys).IndexOf(follower);
            if (idx < 0 || idx >= T.Count)
            {
                return Vec3.Zero;
            }

            Vec3 direction = Vec3.Normalize(offsets[follower]);
            float alpha = 0.2f;
            float tension = (float)Math.Max(0.0, T[idx]);

            return direction * (tension * alpha);
        }

        private static MathMat BuildRotation(float roll, float pitch, float yaw)
        {
            double r = roll * SwarmConstants.Deg2Rad;
            double p = pitch * SwarmConstants.Deg2Rad;
            double y = yaw * SwarmConstants.Deg2Rad;

            double cr = Math.Cos(r), sr = Math.Sin(r);
            double cp = Math.Cos(p), sp = Math.Sin(p);
            double cy = Math.Cos(y), sy = Math.Sin(y);

            return MathMat.Build.DenseOfArray(new double[,] {
                { cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr },
                { sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr },
                { -sp,   cp*sr,             cp*cr            }
            });
        }

        private static MathVec Cross(MathVec a, MathVec b)
            => MathVec.Build.Dense(new double[] {
                a[1]*b[2] - a[2]*b[1],
                a[2]*b[0] - a[0]*b[2],
                a[0]*b[1] - a[1]*b[0]
            });
    }

    // --- Formation Controller ---
    internal class Formation : Swarm
    {
        private readonly Dictionary<MAVState, Vec3> offsets = new Dictionary<MAVState, Vec3>();
        private readonly Dictionary<MAVState, (PIDController Roll, PIDController Pitch)> pids
            = new Dictionary<MAVState, (PIDController, PIDController)>();
        private readonly LoadAttitudeController loadCtrl = new LoadAttitudeController();

        private PointLatLngAlt masterPos;
        private DateTime lastTimeUtc = DateTime.UtcNow;

        // Adaptive control constants
        private const float Sigma = 0.1f;
        private const float Kp0 = 1f, Ki0 = 0.1f, Kd0 = 0.01f;
        private const float Kv0 = 1f;
        private const float Gi = 0.1f, Gd = 0.01f;

        public void SetOffset(MAVState m, Vec3 offset)
            => offsets[m] = offset;

        public Vec3 GetOffset(MAVState m)
            => offsets.TryGetValue(m, out var off) ? off : Vec3.Zero;

        // Backward-compatible aliases
        public void setOffsets(MAVState m, Vec3 offset) => SetOffset(m, offset);
        public Vec3 getOffsets(MAVState m) => GetOffset(m);

        public override void Update()
        {
            var mav = MainV2.comPort.MAV;
            if (mav.cs.lat == 0 || mav.cs.lng == 0) return;
            if (Leader == null) Leader = mav;

            masterPos = new PointLatLngAlt(Leader.cs.lat, Leader.cs.lng, Leader.cs.alt, "");
            lastTimeUtc = DateTime.UtcNow;
        }

        public override void SendCommand()
        {
            if (masterPos.Lat == 0 || masterPos.Lng == 0) return;

            var (lx, ly, zone, north) = GeoHelper.LatLonToUtm(masterPos.Lat, masterPos.Lng);
            float heading = -Leader.cs.yaw * (float)(Math.PI / 180.0);

            foreach (var port in MainV2.Comports)
            {
                foreach (var mav in port.MAVlist)
                {
                    if (mav == Leader) continue;

                    Vec3 comp = loadCtrl.Compensate(Leader, mav, offsets, out _);
                    Vec3 off = offsets[mav];

                    double x2 = lx + off.X * Math.Cos(heading) - off.Y * Math.Sin(heading);
                    double y2 = ly + off.X * Math.Sin(heading) + off.Y * Math.Cos(heading);
                    var (tlat, tlon) = GeoHelper.UtmToLatLon(x2, y2, zone, north);
                    double talt = masterPos.Alt + off.Z;

                    if (!pids.ContainsKey(mav))
                        pids[mav] = (new PIDController(Kp0, Ki0, Kd0), new PIDController(Kv0, Ki0, Kd0));

                    var (pidP, pidV) = pids[mav];

                    Vec3 curPos = new Vec3((float)mav.cs.lat, (float)mav.cs.lng, (float)mav.cs.alt);
                    Vec3 tarPos = new Vec3((float)tlat, (float)tlon, (float)talt);
                    Vec3 posErr = tarPos - curPos;
                    Vec3 curVel = new Vec3((float)mav.cs.vx, (float)mav.cs.vy, (float)mav.cs.vz);
                    Vec3 velErr = -curVel;

                    float dt = (float)(DateTime.UtcNow - lastTimeUtc).TotalSeconds;

                    // Adapt gains
                    pidP.Kp += Sigma * (pidP.Kp - Kp0) + Gi * posErr.LengthSquared();
                    pidP.Ki += Sigma * (pidP.Ki - Ki0) + Gi * posErr.LengthSquared();
                    pidP.Kd += Sigma * (pidP.Kd - Kd0) + Gd * velErr.LengthSquared();
                    pidV.Kp += Sigma * (pidV.Kp - Kv0) + Gi * velErr.LengthSquared();
                    pidV.Ki += Sigma * (pidV.Ki - Ki0) + Gi * velErr.LengthSquared();
                    pidV.Kd += Sigma * (pidV.Kd - Kd0) + Gd * velErr.LengthSquared();

                    pidP.ClampLower(Kp0, Ki0, Kd0);
                    pidV.ClampLower(Kv0, Ki0, Kd0);

                    float dPos = (posErr.X - pidP.LastError) / dt;
                    float outP = pidP.Step(posErr.X, dPos, dt);
                    pidP.LastError = posErr.X;

                    float dVel = (velErr.X - pidV.LastError) / dt;
                    float outV = pidV.Step(velErr.X, dVel, dt);
                    pidV.LastError = velErr.X;

                    // Final command
                    float cmd = outP + outV + comp.X;

                    Vec3 leaderVel = new Vec3((float)Leader.cs.vx, (float)Leader.cs.vy, (float)Leader.cs.vz);

                    port.setPositionTargetGlobalInt(
                        mav.sysid,
                        mav.compid,
                        true, true, false, false,
                        MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT_INT,
                        (float)tlat, (float)tlon, (float)talt,
                        leaderVel.X, leaderVel.Y, leaderVel.Z,
                        0f, 0f);
                }
            }
            lastTimeUtc = DateTime.UtcNow;
        }
    }
}
