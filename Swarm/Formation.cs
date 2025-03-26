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
    // Global swarm constants
    public static class SwarmConstants
    {
        public const float DEFAULT_LEADER_MASS = 1.0f;
    }

    /// <summary>
    /// Formation controller that follows a leader.
    /// Uses original coordinate transformation and follower targeting,
    /// augmented with adaptive control, load attitude, and tension correction.
    /// </summary>
    class Formation : Swarm
    {
        // Original offset dictionary (per MAV)
        Dictionary<MAVState, Vector3> offsets = new Dictionary<MAVState, Vector3>();
        // Retain original PID dictionary (for legacy or non-adaptive usage)
        private Dictionary<MAVState, Tuple<PID, PID, PID, PID>> pids =
            new Dictionary<MAVState, Tuple<PID, PID, PID, PID>>();

        // New controllers and timing for adaptive control
        private Dictionary<MAVState, AdaptiveFormationController> controllers =
            new Dictionary<MAVState, AdaptiveFormationController>();
        private Dictionary<MAVState, DateTime> timestamps =
            new Dictionary<MAVState, DateTime>();

        // Updated leader position container
        private PointLatLngAlt masterpos = new PointLatLngAlt();
        // For coordinate transformation (WGS84 to UTM)
        CoordinateTransformationFactory ctfac = new CoordinateTransformationFactory();
        IGeographicCoordinateSystem wgs84 = GeographicCoordinateSystem.WGS84;

        // Parameters for collision avoidance and load compensation
        private float minSeparation = 5.0f;      // meters
        private float avoidanceGain = 5.0f;      // gain for repulsion

        // New adaptive controllers
        private LoadAttitudeController attitudeController = new LoadAttitudeController();
        private TensionSolver tensionSolver = new TensionSolver();

        /// <summary>
        /// Sets the formation offset for a given MAV.
        /// </summary>
        public void setOffsets(MAVState mav, double x, double y, double z)
        {
            offsets[mav] = new Vector3((float)x, (float)y, (float)z);
            log.Info(mav.ToString() + " " + offsets[mav].ToString());
        }

        /// <summary>
        /// Gets the formation offset for a given MAV.
        /// </summary>
        public Vector3 getOffsets(MAVState mav)
        {
            if (offsets.ContainsKey(mav))
            {
                return offsets[mav];
            }
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

        // Helper to wrap angles to the [-180, 180] range.
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

            // For ArduPlane control, we use adaptive control components.
            // First, update leader transformation (using a fixed UTM zone calculation as in the original)
            int utmzone = (int)((masterpos.Lng - -186.0) / 6.0);
            IProjectedCoordinateSystem utm = ProjectedCoordinateSystem.WGS84_UTM(utmzone, masterpos.Lat >= 0);
            ICoordinateTransformation trans = ctfac.CreateFromCoordinateSystems(wgs84, utm);

            // Convert leader's GPS to UTM once.
            double[] pLeaderBase = trans.MathTransform.Transform(new double[] { Leader.cs.lng, Leader.cs.lat });

            // Iterate over all communication ports and MAVs.
            foreach (var port in MainV2.Comports.ToArray())
            {
                foreach (var mav in port.MAVlist)
                {
                    if (mav == Leader)
                        continue;

                    // Compute desired follower target in UTM coordinates.
                    Vector3 offset = getOffsets(mav);
                    // Rotate offset according to leader's heading.
                    double heading = -Leader.cs.yaw * MathHelper.deg2rad;
                    double dx = offset.x * Math.Cos(heading) - offset.y * Math.Sin(heading);
                    double dy = offset.x * Math.Sin(heading) + offset.y * Math.Cos(heading);

                    // Desired position is leader's UTM plus rotated offset.
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

                    // Reconstruct desired position as UTM vector with altitude adjusted.
                    Vector3 desiredPosition = new Vector3((float)pLeader[0], (float)pLeader[1], (float)(Leader.cs.alt + offset.z));
                    Vector3 desiredVelocity = new Vector3((float)Leader.cs.vx, (float)Leader.cs.vy, (float)Leader.cs.vz);

                    // Follower current position in UTM.
                    Vector3 followerPos = new Vector3((float)pFollower[0], (float)pFollower[1], (float)mav.cs.alt);

                    // Compute position and velocity errors.
                    Vector3 posError = desiredPosition - followerPos;
                    Vector3 velError = desiredVelocity - new Vector3((float)mav.cs.vx, (float)mav.cs.vy, (float)mav.cs.vz);

                    // Collision avoidance (using UTM coordinates)
                    Vector3 avoidance = Vector3.Zero;
                    foreach (var otherPort in MainV2.Comports)
                    {
                        foreach (var other in otherPort.MAVlist)
                        {
                            if (other == mav || other == Leader)
                                continue;

                            double[] posOther;
                            try
                            {
                                posOther = trans.MathTransform.Transform(new double[] { other.cs.lng, other.cs.lat });
                            }
                            catch
                            {
                                continue;
                            }

                            Vector3 rel = new Vector3(
                                (float)(pFollower[0] - posOther[0]),
                                (float)(pFollower[1] - posOther[1]),
                                (float)(mav.cs.alt - other.cs.alt));
                            float dist = (float)Math.Sqrt(rel.x * rel.x + rel.y * rel.y + rel.z * rel.z);

                            if (dist < minSeparation && dist > 0.001f)
                            {
                                float strength = avoidanceGain / (dist * dist);
                                avoidance += VectorUtils.NormalizeVector(rel) * strength;
                            }
                        }
                    }

                    // Manage timing for adaptive control.
                    if (!timestamps.ContainsKey(mav))
                        timestamps[mav] = DateTime.UtcNow;
                    float dt = (float)(DateTime.UtcNow - timestamps[mav]).TotalSeconds;
                    timestamps[mav] = DateTime.UtcNow;

                    // Decide control action based on firmware type.
                    if (mav.cs.firmware == Firmwares.ArduPlane)
                    {
                        // Initialize controller if needed.
                        if (!controllers.ContainsKey(mav))
                            controllers[mav] = new AdaptiveFormationController();

                        // Compute adaptive control (PD style) using position and velocity errors.
                        Vector3 control = controllers[mav].ComputeControl(posError, velError, dt);
                        // Add collision avoidance.
                        control += avoidance;
                        // Add load compensation and tension correction.
                        control += attitudeController.CompensateRigidBodyDynamics(Leader, mav, offsets);
                        control += tensionSolver.ComputeTensionCorrectionBalanced(Leader, mav, offsets);

                        // Compute vertical thrust (with gravity compensation).
                        const float GRAVITY = 9.81f;
                        const float UAV_MASS_KG = 0.7f;
                        const float MAX_THRUST_N = UAV_MASS_KG * GRAVITY * 3.0f;
                        double verticalThrust = control.z + GRAVITY;
                        float thrustCommand = (float)MathHelper.constrain(verticalThrust / MAX_THRUST_N, 0.1, 1);

                        // Create a quaternion from control.x (roll) and control.y (pitch).
                        // (Yaw is set to zero here; additional yaw handling can be added if needed.)
                        Quaternion q = Quaternion.from_euler312(control.x * MathHelper.deg2rad,
                                                                  control.y * MathHelper.deg2rad,
                                                                  0);
                        MAVLink.mavlink_set_attitude_target_t att_target = new MAVLink.mavlink_set_attitude_target_t();
                        att_target.target_system = mav.sysid;
                        att_target.target_component = mav.compid;
                        // Use type_mask to indicate that only thrust and attitude are controlled.
                        att_target.type_mask = 0b00000100;
                        att_target.thrust = thrustCommand;
                        att_target.q = new float[4] { (float)q.q1, (float)q.q2, (float)q.q3, (float)q.q4 };

                        port.sendPacket(att_target, mav.sysid, mav.compid);
                    }
                    else
                    {
                        // For non-ArduPlane firmwares, use the original position/velocity target method.
                        Vector3 vel = new Vector3(Leader.cs.vx, Leader.cs.vy, Leader.cs.vz);
                        port.setPositionTargetGlobalInt(mav.sysid, mav.compid, true,
                            true, false, false,
                            MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT_INT, desiredPosition.x, desiredPosition.y, desiredPosition.z,
                            vel.x, vel.y, vel.z, 0, 0);

                        // Yaw control: if gimbal is not enabled and yaw error is significant, send a yaw command.
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

        // Property to indicate if gimbal control is enabled.
        public bool gimbal { get; set; }
    }

    // PID class remains from the original implementation.
    public class PID
    {
        private float _dt;
        private float M_2PI = (float)(Math.PI * 2);
        private float _input;
        private float _derivative;
        private float _kp;
        private float _ki;
        private float _integrator;
        private float _imax;
        private float _kd;
        private float _ff;
        private float _filt_hz = AC_PID_FILT_HZ_DEFAULT;

        const float AC_PID_FILT_HZ_DEFAULT = 20.0f;
        const float AC_PID_FILT_HZ_MIN = 0.01f;

        public PID(float initial_p, float initial_i, float initial_d, float initial_imax, float initial_filt_hz, float dt, float initial_ff)
        {
            _dt = dt;
            _integrator = 0.0f;
            _input = 0.0f;
            _derivative = 0.0f;

            _kp = initial_p;
            _ki = initial_i;
            _kd = initial_d;
            _imax = Math.Abs(initial_imax);
            filt_hz(initial_filt_hz);
            _ff = initial_ff;

            _flags._reset_filter = true;
        }

        public void set_dt(float dt)
        {
            _dt = dt;
        }

        public void filt_hz(float hz)
        {
            _filt_hz = hz;
            _filt_hz = Math.Max(_filt_hz, AC_PID_FILT_HZ_MIN);
        }

        public void set_input_filter_all(float input)
        {
            if (!isfinite(input))
                return;

            if (_flags._reset_filter)
            {
                _flags._reset_filter = false;
                _input = input;
                _derivative = 0.0f;
            }

            float input_filt_change = get_filt_alpha() * (input - _input);
            _input = _input + input_filt_change;
            if (_dt > 0.0f)
                _derivative = input_filt_change / _dt;
        }

        private bool isfinite(float input)
        {
            return !float.IsInfinity(input);
        }

        public float get_p()
        {
            _pid_info.P = (_input * _kp);
            return _pid_info.P;
        }

        public float get_i()
        {
            if (!is_zero(_ki) && !is_zero(_dt))
            {
                _integrator += (_input * _ki) * _dt;
                if (_integrator < -_imax)
                    _integrator = -_imax;
                else if (_integrator > _imax)
                    _integrator = _imax;
                _pid_info.I = _integrator;
                return _integrator;
            }
            return 0;
        }

        public float get_d()
        {
            _pid_info.D = (_kd * _derivative);
            return _pid_info.D;
        }

        public float get_ff(float requested_rate)
        {
            _pid_info.FF = requested_rate * _ff;
            return _pid_info.FF;
        }

        public float get_pi()
        {
            return get_p() + get_i();
        }

        public float get_pid()
        {
            return get_p() + get_i() + get_d();
        }

        public void reset_I()
        {
            _integrator = 0;
        }

        public float get_filt_alpha()
        {
            if (is_zero(_filt_hz))
                return 1.0f;
            float rc = 1 / (M_2PI * _filt_hz);
            return _dt / (_dt + rc);
        }

        private bool is_zero(float value)
        {
            return value == 0;
        }

        internal class flags
        {
            internal bool _reset_filter;
        }
        flags _flags = new flags();

        internal class pid_info
        {
            internal float P;
            internal float I;
            internal float D;
            internal float FF;
        }
        pid_info _pid_info = new pid_info();
    }

    // Adaptive controller that updates gains based on position and velocity errors.
    public class AdaptiveFormationController
    {
        private Matrix<double> Kp = Matrix<double>.Build.DenseIdentity(3);
        private Matrix<double> Kv = Matrix<double>.Build.DenseIdentity(3);
        private readonly double sigma = 0.02;   // Adaptive rate
        private readonly double gammaP = 0.05;    // Proportional gain update rate
        private readonly double gammaV = 0.05;    // Derivative gain update rate

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

            // Clamp gains within a specified range.
            Kp = Kp.Map(x => MathHelper.constrain(x, 0.5, 5.0));
            Kv = Kv.Map(x => MathHelper.constrain(x, 0.5, 5.0));

            var control = -(Kp * xi + Kv * zeta);
            return new Vector3((float)control[0], (float)control[1], (float)control[2]);
        }
    }

    // Compensates for rigid-body load dynamics.
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

    // Computes tension corrections using a pseudoinverse-based method.
    public class TensionSolver
    {
        private Vector<double> lambda;

        public TensionSolver()
        {
            // Initialize lambda vector with default values.
            lambda = Vector<double>.Build.Dense(3, 2.0);
        }

        public void SetLambda(Vector<double> newLambda)
        {
            lambda = newLambda;
        }

        private Matrix<double> ComputeNullspace(Matrix<double> Phi)
        {
            var svd = Phi.Svd(true);
            return svd.VT.SubMatrix(svd.Rank, svd.VT.RowCount - svd.Rank, 0, svd.VT.ColumnCount).Transpose();
        }

        public Vector3 ComputeTensionCorrectionBalanced(MAVState leader, MAVState follower, Dictionary<MAVState, Vector3> offsets)
        {
            var W = Vector<double>.Build.DenseOfArray(new double[]
            {
                leader.cs.ax,
                leader.cs.ay,
                leader.cs.az + 9.81,
                0,
                0,
                0
            });

            var attachmentPoints = new List<Vector3>(offsets.Values);
            int n = attachmentPoints.Count;
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
            var N = ComputeNullspace(Phi);
            var T = PhiPlus * W;

            if (N.ColumnCount == lambda?.Count)
                T += N * lambda;

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

    // Helper methods for vector normalization.
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
