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
        public const double PayloadMass = 1.0; // fixed mass
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
        public const float PositionGain = 1.0f;
    }

    /// <summary>Math utilities.</summary>
    internal static class MathUtils
    {
        public const double Deg2Rad = Math.PI / 180.0;
        public static double Constrain(double val, double min, double max) => val < min ? min : (val > max ? max : val);
        public static double WrapDegrees(double ang)
        {
            if (ang > 180) ang -= 360;
            else if (ang < -180) ang += 360;
            return ang;
        }
    }

    /// <summary>Extensions for vectors.</summary>
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

    /// <summary>Utilities for Vector3.</summary>
    internal static class VectorUtils
    {
        public static Vector3 Normalize(Vector3 v)
        {
            double len = Math.Sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
            return len < 1e-6 ? Vector3.Zero : new Vector3((float)(v.x/len), (float)(v.y/len), (float)(v.z/len));
        }
    }

    /// <summary>Controller for payload dynamics and tension distribution.</summary>
    internal class LoadAttitudeController
    {
        private readonly Dictionary<int, float> lastYawSpeed = new Dictionary<int, float>();
        private double compGainHat = 1.0;
        public double CompGainHat => compGainHat;

        public Matrix<double> ComputeMappingMatrix(Dictionary<MAVState, Vector3> offsets)
        {
            int n = offsets.Count;
            var Phi = Matrix<double>.Build.Dense(6, 3*n);
            var pts = new List<Vector3>(offsets.Values);
            for(int i=0;i<n;i++)
            {
                Vector3 r = pts[i];
                var q = VectorUtils.Normalize(r);
                for(int ax=0;ax<3;ax++) Phi[ax,3*i+ax] = q[ax];
                var hat = Matrix<double>.Build.DenseOfArray(new double[,] {
                    {0,-r.z,r.y},{r.z,0,-r.x},{-r.y,r.x,0}
                });
                var mvec = hat * Vector<double>.Build.Dense(new[]{q.x,q.y,q.z});
                for(int ax=0;ax<3;ax++)
                    for(int c=0;c<3;c++)
                        Phi[3+ax,3*i+c] = mvec[ax];
            }
            return Phi;
        }

        public Vector<double> ComputePayloadWrench(MAVState leader)
        {
            double mL = FormationConstants.PayloadMass;
            var aL = Vector<double>.Build.Dense(new[]
            {
                leader.cs.ax*mL,
                leader.cs.ay*mL,
                (leader.cs.az+(float)FormationConstants.Gravity)*mL
            });
            var R = BuildRotation(leader.cs.roll,leader.cs.pitch,leader.cs.yaw);
            var Wf = -R.Transpose()*aL;
            var Wm = Vector<double>.Build.Dense(3); // assume no moment term
            var W = Vector<double>.Build.Dense(6);
            W.SetSubVector(0,3,Wf);
            W.SetSubVector(3,3,Wm);
            return W;
        }

        public Vector<double> ComputeTensions(Matrix<double> Phi, Vector<double> W)
            => Phi.PseudoInverse()*W;

        public void UpdateCompGain(Vector3 rawComp, Vector3 error, float dt)
        {
            double dot = rawComp.x*error.x + rawComp.y*error.y + rawComp.z*error.z;
            compGainHat += FormationConstants.AdaptationGain*dot*dt;
            compGainHat = MathUtils.Constrain(compGainHat,FormationConstants.MinCompGain,FormationConstants.MaxCompGain);
        }

        private Matrix<double> BuildRotation(float roll, float pitch, float yaw)
        {
            double r=roll*MathUtils.Deg2Rad, p=pitch*MathUtils.Deg2Rad, y=yaw*MathUtils.Deg2Rad;
            double cr=Math.Cos(r), sr=Math.Sin(r), cp=Math.Cos(p), sp=Math.Sin(p), cy=Math.Cos(y), sy=Math.Sin(y);
            return Matrix<double>.Build.DenseOfArray(new double[,] {
                {cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr},
                {sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr},
                {-sp, cp*sr, cp*cr}
            });
        }
    }

    /// <summary>Formation controller.</summary>
    internal class Formation : Swarm
    {
        private readonly Dictionary<MAVState, Vector3> offsets = new Dictionary<MAVState, Vector3>();
        private readonly Dictionary<MAVState, Vector3> desiredAccel = new Dictionary<MAVState, Vector3>();
        private readonly LoadAttitudeController ctrl = new LoadAttitudeController();

        /// <summary>
        /// Ensure Leader is always set.
        /// </summary>
        public override void Update()
        {
            if (MainV2.comPort != null && MainV2.comPort.MAV != null)
            {
                Leader = MainV2.comPort.MAV;
            }
        }

        /// <summary>Set desired acceleration for MAV.</summary>
        public void SetDesiredAcceleration(MAVState mav, Vector3 accel)
            => desiredAccel[mav] = accel;

        /// <summary>Set attachment offset.</summary>
        public void SetFormationOffset(MAVState mav, double x, double y, double z)
            => offsets[mav] = new Vector3((float)x, (float)y, (float)z);

        /// <summary>Alias for legacy code: lowercase setOffsets.</summary>
        public void setOffsets(MAVState mav, double x, double y, double z)
            => SetFormationOffset(mav, x, y, z);

        /// <summary>Retrieve a follower's offset.</summary>
        public Vector3 getOffsets(MAVState mav)
            => offsets.TryGetValue(mav, out var off) ? off : Vector3.Zero;

        /// <summary>Compute a simple P‐control to maintain offset above payload.</summary>
        private Vector3 ComputeOffsetTrackingAccel(MAVState mav)
        {
            // Leader's reference position
            double lat0 = Leader.cs.lat;
            double lng0 = Leader.cs.lng;
            double alt0 = Leader.cs.alt;

            // Transform to UTM
            int zone = (int)((lng0 + 180) / 6);
            var utm = ProjectedCoordinateSystem.WGS84_UTM(zone, lat0 >= 0);
            var trans = new CoordinateTransformationFactory()
                .CreateFromCoordinateSystems(GeographicCoordinateSystem.WGS84, utm);

            var leaderXY = trans.MathTransform.Transform(new[] { lng0, lat0 });
            var off = offsets.TryGetValue(mav, out var o) ? o : Vector3.Zero;
            double hdg = -Leader.cs.yaw * MathUtils.Deg2Rad;
            double desX = leaderXY[0] + off.x * Math.Cos(hdg) - off.y * Math.Sin(hdg);
            double desY = leaderXY[1] + off.x * Math.Sin(hdg) + off.y * Math.Cos(hdg);
            double desZ = alt0 + off.z;

            var currXY = trans.MathTransform.Transform(new[] { mav.cs.lng, mav.cs.lat });
            double errX = desX - currXY[0];
            double errY = desY - currXY[1];
            double errZ = desZ - mav.cs.alt;

            return new Vector3((float)(FormationConstants.PositionGain * errX),
                               (float)(FormationConstants.PositionGain * errY),
                               (float)(FormationConstants.PositionGain * errZ));
        }

        /// <summary>Send commands to each follower.</summary>
        public override void SendCommand()
        {
            if (Leader == null) return;
            const float dt = 0.02f;

            foreach (var port in MainV2.Comports)
            foreach (var mav in port.MAVlist)
            {
                if (mav == Leader) continue;

                // 1. Feed-forward accel
                var u_ff = desiredAccel.TryGetValue(mav, out var ff) ? ff : ComputeOffsetTrackingAccel(mav);

                // 2. Payload tension compensation
                var Phi = ctrl.ComputeMappingMatrix(offsets);
                var W = ctrl.ComputePayloadWrench(Leader);
                var T = ctrl.ComputeTensions(Phi, W);
                int idx = new List<MAVState>(offsets.Keys).IndexOf(mav);
                float Ti = idx >= 0 ? (float)MathUtils.Constrain((float)T[idx], 0, float.MaxValue) : 0f;
                var q_i = offsets.TryGetValue(mav, out var o2) ? VectorUtils.Normalize(o2) : Vector3.Zero;
                var rawComp = q_i * Ti;

                // 3. Adaptive compensation
                var actual = new Vector3((float)mav.cs.ax, (float)mav.cs.ay, (float)mav.cs.az);
                var comp = rawComp * (float)ctrl.CompGainHat;
                var error = actual - (u_ff + comp);
                ctrl.UpdateCompGain(rawComp, error, dt);

                // 4. Command accel
                var cmdAccel = u_ff + comp;

                // 5. Send target
                SendPositionOrAttitude(port, mav, cmdAccel);
            }
        }

        /// <summary>Map acceleration to attitude or position target.</summary>
        private void SendPositionOrAttitude(dynamic port, MAVState mav, Vector3 accel)
        {
            double yawErr = new PointLatLngAlt(Leader.cs.lat, Leader.cs.lng, Leader.cs.alt)
                .GetBearing(mav.cs.Location) - mav.cs.yaw;
            double yawRad = MathUtils.WrapDegrees(yawErr) * MathUtils.Deg2Rad;

            if (mav.cs.firmware == Firmwares.ArduPlane)
            {
                float roll = (float)Math.Atan2(accel.x, (float)FormationConstants.Gravity);
                float pitch = (float)Math.Atan2(-accel.y, (float)FormationConstants.Gravity);
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
                    Leader.cs.lat, Leader.cs.lng, Leader.cs.alt,
                    accel.x, accel.y, accel.z,
                    0, 0);
            }
        }
    }
}
