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

    // Main formation class
    class Formation : Swarm
    {
        const float UAV_MASS_KG = 0.7f;
        const float GRAVITY = 9.81f;
        const float MAX_THRUST_N = UAV_MASS_KG * GRAVITY * 3.0f; // 3x hover thrust (~20.6 N)

        Dictionary<MAVState, Vector3> offsets = new Dictionary<MAVState, Vector3>();
        private Dictionary<MAVState, AdaptiveFormationController> controllers = new Dictionary<MAVState, AdaptiveFormationController>();
        private Dictionary<MAVState, DateTime> timestamps = new Dictionary<MAVState, DateTime>();
        private PointLatLngAlt masterpos = new PointLatLngAlt();
        private DateTime lastUpdate = DateTime.UtcNow;
        private LoadAttitudeController attitudeController = new LoadAttitudeController();
        private TensionSolver tensionSolver = new TensionSolver();
        private float minSeparation = 0.00002f;
        private float avoidanceGain = 0.0001f;

        public void setOffsets(MAVState mav, double x, double y, double z)
            => offsets[mav] = new Vector3((float)x, (float)y, (float)z);

        public Vector3 getOffsets(MAVState mav)
        {
            if (offsets.ContainsKey(mav))
                return offsets[mav];
            return new Vector3((float)offsets.Count, 0, 0);
        }

        public override void Update()
        {
            if (MainV2.comPort.MAV.cs.lat == 0 || MainV2.comPort.MAV.cs.lng == 0)
                return;
            if (Leader == null)
                Leader = MainV2.comPort.MAV;
            masterpos = new PointLatLngAlt(Leader.cs.lat, Leader.cs.lng, Leader.cs.alt, "");
        }

        private void UpdateAdaptiveLambda()
        {
            var followers = new List<MAVState>(offsets.Keys);
            int n = followers.Count;
            if (n < 3) return; // At least 3 UAVs are needed for rigid-body payload stabilization

            double[] pitchLoads = new double[n];
            for (int i = 0; i < n; i++)
            {
                // Replace with actual load calculations if needed.
                pitchLoads[i] = 0.0;
            }

            double total = 0;
            foreach (var l in pitchLoads)
                total += l;
            if (total == 0)
                total = 1; // avoid division by zero

            double[] shares = new double[n];
            for (int i = 0; i < n; i++)
                shares[i] = 1.0 - (pitchLoads[i] / total);

            int lambdaSize = n * (n - 1) / 2;
            var lambda = Vector<double>.Build.Dense(lambdaSize);
            int k = 0;
            for (int i = 0; i < n; i++)
            {
                for (int j = i + 1; j < n; j++)
                {
                    lambda[k++] = shares[i] + shares[j];
                }
            }

            tensionSolver.SetLambda(lambda);
        }

        public override void SendCommand()
        {
            if (masterpos.Lat == 0 || masterpos.Lng == 0)
                return;

            UpdateAdaptiveLambda();

            foreach (var port in MainV2.Comports.ToArray())
            {
                foreach (var mav in port.MAVlist)
                {
                    if (mav == Leader)
                        continue;

                    if (!controllers.ContainsKey(mav))
                        controllers[mav] = new AdaptiveFormationController();

                    Vector3 offset = getOffsets(mav);
                    Vector3 desiredPosition, desiredVelocity;

                    int utmzone = (int)((Leader.cs.lng - -186.0) / 6.0);
                    IProjectedCoordinateSystem utm = ProjectedCoordinateSystem.WGS84_UTM(utmzone, Leader.cs.lat >= 0);
                    CoordinateTransformationFactory ctfac = new CoordinateTransformationFactory();
                    IGeographicCoordinateSystem wgs84 = GeographicCoordinateSystem.WGS84;
                    ICoordinateTransformation trans = ctfac.CreateFromCoordinateSystems(wgs84, utm);

                    double[] pll = new double[] { Leader.cs.lng, Leader.cs.lat };
                    double[] pLeader;
                    try { pLeader = trans.MathTransform.Transform(pll); }
                    catch { return; }

                    var heading = -Leader.cs.yaw * MathHelper.deg2rad;
                    var dx = offset.x * Math.Cos(heading) - offset.y * Math.Sin(heading);
                    var dy = offset.x * Math.Sin(heading) + offset.y * Math.Cos(heading);

                    pLeader[0] += dx;
                    pLeader[1] += dy;

                    double[] pFollower;
                    try { pFollower = trans.MathTransform.Transform(new double[] { mav.cs.lng, mav.cs.lat }); }
                    catch { return; }

                    desiredPosition = new Vector3((float)pLeader[0], (float)pLeader[1], (float)(Leader.cs.alt + offset.z));
                    desiredVelocity = new Vector3((float)Leader.cs.vx, (float)Leader.cs.vy, (float)Leader.cs.vz);

                    Vector3 followerPos = new Vector3((float)pFollower[0], (float)pFollower[1], (float)mav.cs.alt);
                    Vector3 posError = desiredPosition - followerPos;
                    Vector3 velError = desiredVelocity - new Vector3((float)mav.cs.vx, (float)mav.cs.vy, (float)mav.cs.vz);

                    Vector3 avoidance = Vector3.Zero;
                    foreach (var otherPort in MainV2.Comports)
                    {
                        foreach (var other in otherPort.MAVlist)
                        {
                            if (other == mav || other == Leader)
                                continue;

                            Vector3 rel = new Vector3(
                                (float)(mav.cs.lat - other.cs.lat),
                                (float)(mav.cs.lng - other.cs.lng),
                                (float)(mav.cs.alt - other.cs.alt));
                            float dist = (float)Math.Sqrt(rel.x * rel.x + rel.y * rel.y + rel.z * rel.z);
                            if (dist < minSeparation && dist > 0.000001f)
                            {
                                float strength = avoidanceGain / (dist * dist);
                                avoidance += VectorUtils.NormalizeVector(rel) * strength;
                            }
                        }
                    }

                    if (!timestamps.ContainsKey(mav))
                        timestamps[mav] = DateTime.UtcNow;
                    float dt = (float)(DateTime.UtcNow - timestamps[mav]).TotalSeconds;
                    timestamps[mav] = DateTime.UtcNow;

                    Vector3 control = controllers[mav].ComputeControl(posError, velError, dt);
                    control += avoidance;
                    control += attitudeController.CompensateRigidBodyDynamics(Leader, mav, offsets);
                    control += tensionSolver.ComputeTensionCorrectionBalanced(Leader, mav, offsets);

                    MAVLink.mavlink_set_attitude_target_t att_target = new MAVLink.mavlink_set_attitude_target_t();
                    att_target.target_system = mav.sysid;
                    att_target.target_component = mav.compid;
                    att_target.type_mask = 0b00000100;

                    double verticalThrust = control.z + GRAVITY;
                    att_target.thrust = (float)MathHelper.constrain(verticalThrust / MAX_THRUST_N, 0.1, 1);

                    Quaternion q = Quaternion.from_euler312(
                        control.x * MathHelper.deg2rad,
                        control.y * MathHelper.deg2rad, 0);
                    att_target.q = new float[4] { (float)q.q1, (float)q.q2, (float)q.q3, (float)q.q4 };

                    port.sendPacket(att_target, mav.sysid, mav.compid);
                }
            }
        }
    }

    // TensionSolver implements a pseudoinverse-based approach to compute tension corrections.
    public class TensionSolver
    {
        private Vector<double> lambda;

        public TensionSolver()
        {
            // Initialize lambda vector. Adjust size if needed.
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

    // AdaptiveFormationController adapts its gains based on position and velocity errors.
    public class AdaptiveFormationController
    {
        private Matrix<double> Kp = Matrix<double>.Build.DenseIdentity(3);
        private Matrix<double> Kv = Matrix<double>.Build.DenseIdentity(3);
        private readonly double sigma = 0.02; //Adaptive rate
        private readonly double gammaP = 0.05; //Proportional gain update rate
        private readonly double gammaV = 0.05; //Derivative gain update rate

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

            // PD Clamping factor
            Kp = Kp.Map(x => MathHelper.constrain(x, 0.5, 5.0));
            Kv = Kv.Map(x => MathHelper.constrain(x, 0.5, 5.0));

            var control = -(Kp * xi + Kv * zeta);
            return new Vector3((float)control[0], (float)control[1], (float)control[2]);
        }
    }

    // LoadAttitudeController compensates for the rigid-body load dynamics.
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
                leader.cs.ax * leaderMass,
                leader.cs.ay * leaderMass,
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

    // Helper functions for normalizing vectors.
    public static class VectorUtils
    {
        public static Vector3 NormalizeVector(Vector3 v)
        {
            var length = Math.Sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
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
