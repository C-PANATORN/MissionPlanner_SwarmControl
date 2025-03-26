using MissionPlanner.ArduPilot;
using MissionPlanner.Utilities;
using System;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using ProjNet.CoordinateSystems;
using ProjNet.CoordinateSystems.Transformations;
using GeoAPI.CoordinateSystems;
using GeoAPI.CoordinateSystems.Transformations;
using Vector3 = MissionPlanner.Utilities.Vector3;

namespace MissionPlanner.Swarm
{
    // Global swarm constants.
    public static class SwarmConstants
    {
        public const float DEFAULT_LEADER_MASS = 1.0f;
    }

    /// <summary>
    /// Formation controller that follows a leader.
    /// This version uses the original formation logic with coordinate transformation,
    /// and implements adaptive control and rigid-body load compensation via the 
    /// AdaptiveFormationController and LoadAttitudeController.
    /// </summary>
    class Formation : Swarm
    {
        // Dictionary of formation offsets (per MAV).
        Dictionary<MAVState, Vector3> offsets = new Dictionary<MAVState, Vector3>();

        // Adaptive control and timing for each follower.
        private Dictionary<MAVState, AdaptiveFormationController> controllers = 
            new Dictionary<MAVState, AdaptiveFormationController>();
        private Dictionary<MAVState, DateTime> timestamps = 
            new Dictionary<MAVState, DateTime>();

        private PointLatLngAlt masterpos = new PointLatLngAlt();

        // Coordinate transformation objects.
        CoordinateTransformationFactory ctfac = new CoordinateTransformationFactory();
        IGeographicCoordinateSystem wgs84 = GeographicCoordinateSystem.WGS84;

        // Load attitude controller instance.
        private LoadAttitudeController attitudeController = new LoadAttitudeController();

        /// <summary>
        /// Sets the formation offset for a given MAV.
        /// </summary>
        public void setOffsets(MAVState mav, double x, double y, double z)
        {
            offsets[mav] = new Vector3((float)x, (float)y, (float)z);
            log.Info(mav.ToString() + " " + offsets[mav].ToString());
        }

        /// <summary>
        /// Retrieves the formation offset for a given MAV.
        /// </summary>
        public Vector3 getOffsets(MAVState mav)
        {
            if (offsets.ContainsKey(mav))
                return offsets[mav];
            return new Vector3(offsets.Count, 0, 0);
        }

        public override void Update()
        {
            if (MainV2.comPort.MAV.cs.lat == 0 || MainV2.comPort.MAV.cs.lng == 0)
                return;

            if (Leader == null)
                Leader = MainV2.comPort.MAV;

            masterpos = new PointLatLngAlt(Leader.cs.lat, Leader.cs.lng, Leader.cs.alt, "");
        }

        // Helper to wrap an angle between -180 and 180 degrees.
        double wrap_180(double input)
        {
            if (input > 180)
                return input - 360;
            if (input < -180)
                return input + 360;
            return input;
        }

        public override void SendCommand()
        {
            if (masterpos.Lat == 0 || masterpos.Lng == 0)
                return;

            // Compute UTM transformation based on the leader's position.
            int utmzone = (int)((masterpos.Lng - -186.0) / 6.0);
            IProjectedCoordinateSystem utm = ProjectedCoordinateSystem.WGS84_UTM(utmzone, masterpos.Lat >= 0);
            ICoordinateTransformation trans = ctfac.CreateFromCoordinateSystems(wgs84, utm);

            // Convert leader's GPS to UTM once.
            double[] pLeaderBase = trans.MathTransform.Transform(new double[] { Leader.cs.lng, Leader.cs.lat });

            foreach (var port in MainV2.Comports.ToArray())
            {
                foreach (var mav in port.MAVlist)
                {
                    if (mav == Leader)
                        continue;

                    // Compute desired follower target in UTM coordinates.
                    Vector3 offset = getOffsets(mav);
                    double heading = -Leader.cs.yaw * MathHelper.deg2rad;
                    double dx = offset.x * Math.Cos(heading) - offset.y * Math.Sin(heading);
                    double dy = offset.x * Math.Sin(heading) + offset.y * Math.Cos(heading);

                    double[] pLeader = new double[2];
                    pLeader[0] = pLeaderBase[0] + dx;
                    pLeader[1] = pLeaderBase[1] + dy;

                    // Convert follower's current GPS position to UTM.
                    double[] pFollower;
                    try
                    {
                        pFollower = trans.MathTransform.Transform(new double[] { mav.cs.lng, mav.cs.lat });
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine("Failed to transform follower position for " + mav.ToString() + "\n" + ex.ToString());
                        continue;
                    }

                    // Desired UTM position and velocity.
                    Vector3 desiredPosition = new Vector3((float)pLeader[0], (float)pLeader[1], (float)(Leader.cs.alt + offset.z));
                    Vector3 desiredVelocity = new Vector3((float)Leader.cs.vx, (float)Leader.cs.vy, (float)Leader.cs.vz);

                    // Follower's current UTM position.
                    Vector3 followerPos = new Vector3((float)pFollower[0], (float)pFollower[1], (float)mav.cs.alt);

                    // Compute position and velocity errors.
                    Vector3 posError = desiredPosition - followerPos;
                    Vector3 velError = desiredVelocity - new Vector3((float)mav.cs.vx, (float)mav.cs.vy, (float)mav.cs.vz);

                    // Compute time step.
                    if (!timestamps.ContainsKey(mav))
                        timestamps[mav] = DateTime.UtcNow;
                    float dt = (float)(DateTime.UtcNow - timestamps[mav]).TotalSeconds;
                    timestamps[mav] = DateTime.UtcNow;

                    // For ArduPlane, use adaptive control and load attitude compensation.
                    if (mav.cs.firmware == Firmwares.ArduPlane)
                    {
                        if (!controllers.ContainsKey(mav))
                            controllers[mav] = new AdaptiveFormationController();

                        Vector3 control = controllers[mav].ComputeControl(posError, velError, dt);
                        control += attitudeController.CompensateRigidBodyDynamics(Leader, mav, offsets);

                        // Compute vertical thrust (with gravity compensation).
                        const float GRAVITY = 9.81f;
                        const float UAV_MASS_KG = 0.7f;
                        const float MAX_THRUST_N = UAV_MASS_KG * GRAVITY * 3.0f;
                        double verticalThrust = control.z + GRAVITY;
                        float thrustCommand = (float)MathHelper.constrain(verticalThrust / MAX_THRUST_N, 0.1, 1);

                        // Create a quaternion from roll (control.x) and pitch (control.y); yaw is set to 0.
                        Quaternion q = Quaternion.from_euler312(control.x * MathHelper.deg2rad,
                                                                  control.y * MathHelper.deg2rad,
                                                                  0);
                        MAVLink.mavlink_set_attitude_target_t att_target = new MAVLink.mavlink_set_attitude_target_t();
                        att_target.target_system = mav.sysid;
                        att_target.target_component = mav.compid;
                        att_target.type_mask = 0b00000100; // Only thrust and attitude are controlled.
                        att_target.thrust = thrustCommand;
                        att_target.q = new float[4] { (float)q.q1, (float)q.q2, (float)q.q3, (float)q.q4 };

                        port.sendPacket(att_target, mav.sysid, mav.compid);
                    }
                    else
                    {
                        // For non-ArduPlane firmwares, use the original position/velocity target approach.
                        Vector3 vel = new Vector3(Leader.cs.vx, Leader.cs.vy, Leader.cs.vz);
                        port.setPositionTargetGlobalInt(mav.sysid, mav.compid, true,
                            true, false, false,
                            MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT_INT, desiredPosition.x, desiredPosition.y, desiredPosition.z,
                            vel.x, vel.y, vel.z, 0, 0);

                        if (!gimbal)
                        {
                            if (Math.Abs(mav.cs.yaw - Leader.cs.yaw) > 3)
                                port.doCommand(mav.sysid, mav.compid, MAVLink.MAV_CMD.CONDITION_YAW, Leader.cs.yaw,
                                    100.0f, 0, 0, 0, 0, 0, false);
                        }
                        else
                        {
                            if (Math.Abs(mav.cs.yaw - Leader.cs.yaw) > 3)
                                port.setMountControl(mav.sysid, mav.compid, 45, 0, Leader.cs.yaw, false);
                        }
                    }
                }
            }
        }

        public bool gimbal { get; set; }
    }

    // Adaptive controller that updates its gains based on position and velocity errors.
    public class AdaptiveFormationController
    {
        private Matrix<double> Kp = Matrix<double>.Build.DenseIdentity(3);
        private Matrix<double> Kv = Matrix<double>.Build.DenseIdentity(3);
        private readonly double sigma = 0.02;
        private readonly double gammaP = 0.05;
        private readonly double gammaV = 0.05;

        public Vector3 ComputeControl(Vector3 posError, Vector3 velError, float dt)
        {
            var xi = Vector<double>.Build.DenseOfArray(new double[] { posError.x, posError.y, posError.z });
            var zeta = Vector<double>.Build.DenseOfArray(new double[] { velError.x, velError.y, velError.z });

            var KpDot = -sigma * (Kp - Matrix<double>.Build.DenseIdentity(3)) +
                        gammaP * xi.ToColumnMatrix() * xi.ToRowMatrix();
            var KvDot = -sigma * (Kv - Matrix<double>.Build.DenseIdentity(3)) +
                        gammaV * zeta.ToColumnMatrix() * zeta.ToRowMatrix();

            Kp += KpDot * dt;
            Kv += KvDot * dt;

            Kp = Kp.Map(x => MathHelper.constrain(x, 0.5, 5.0));
            Kv = Kv.Map(x => MathHelper.constrain(x, 0.5, 5.0));

            var control = -(Kp * xi + Kv * zeta);
            return new Vector3((float)control[0], (float)control[1], (float)control[2]);
        }
    }

    // Controller that compensates for rigid-body load dynamics.
    public class LoadAttitudeController
    {
        public Vector3 CompensateRigidBodyDynamics(MAVState leader, MAVState follower, Dictionary<MAVState, Vector3> offsets)
        {
            if (!offsets.ContainsKey(follower))
                return Vector3.Zero;

            var attachmentPoints = new List<Vector3>(offsets.Values);
            int n = attachmentPoints.Count;
            if (n < 3)
                return Vector3.Zero;

            float leaderMass = SwarmConstants.DEFAULT_LEADER_MASS;
            var F = Vector<double>.Build.DenseOfArray(new double[]
            {
                leader.cs.ax,
                leader.cs.ay,
                (leader.cs.az + 9.81f) * leaderMass
            });

            var Tau = Vector<double>.Build.DenseOfArray(new double[] { 0.0, 0.0, 0.0 });
            var W = Vector<double>.Build.Dense(6);
            for (int i = 0; i < 3; i++)
                W[i] = F[i];
            for (int i = 0; i < 3; i++)
                W[i + 3] = Tau[i];

            var Phi = Matrix<double>.Build.Dense(6, n);
            for (int i = 0; i < n; i++)
            {
                var r = attachmentPoints[i];
                var qi = VectorUtils.NormalizeVector(r);
                Phi[0, i] = qi.x;
                Phi[1, i] = qi.y;
                Phi[2, i] = qi.z;
                Phi[3, i] = r.y * qi.z - r.z * qi.y;
                Phi[4, i] = r.z * qi.x - r.x * qi.z;
                Phi[5, i] = r.x * qi.y - r.y * qi.x;
            }

            var PhiPlus = Phi.PseudoInverse();
            var T = PhiPlus * W;
            int idx = new List<MAVState>(offsets.Keys).IndexOf(follower);
            if (idx >= 0 && idx < T.Count)
            {
                var qi = VectorUtils.NormalizeVector(attachmentPoints[idx]);
                float ti = Math.Max(0, (float)T[idx]);
                return qi * ti;
            }
            return Vector3.Zero;
        }
    }

    // Utility class for vector normalization.
    public static class VectorUtils
    {
        public static Vector3 NormalizeVector(Vector3 v)
        {
            double length = Math.Sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
            if (length == 0)
                return new Vector3(0, 0, 0);
            return new Vector3((float)(v.x / length), (float)(v.y / length), (float)(v.z / length));
        }

        public static Vector3 NormalizeVector(float[] v)
        {
            double length = Math.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
            if (length == 0)
                return new Vector3(0, 0, 0);
            return new Vector3((float)(v[0] / length), (float)(v[1] / length), (float)(v[2] / length));
        }

        public static Vector3 NormalizeVector(int[] v)
        {
            double[] doubleArray = Array.ConvertAll(v, item => (double)item);
            return NormalizeVector(doubleArray);
        }

        public static Vector3 NormalizeVector(double[] v)
        {
            double length = Math.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
            if (length == 0)
                return new Vector3(0, 0, 0);
            return new Vector3((float)(v[0] / length), (float)(v[1] / length), (float)(v[2] / length));
        }
    }
}
