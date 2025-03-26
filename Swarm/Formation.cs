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
    class Formation : Swarm
    {
        const float UAV_MASS_KG = 0.7f;
        const float GRAVITY = 9.81f;
        // const float SPECIFIC_THRUST_KG_PER_WATT = 0.0036f; // Defined but unused
        const float MAX_THRUST_N = UAV_MASS_KG * GRAVITY * 3.0f; // 3x hover thrust (~20.6 N)
        const float DEFAULT_LEADER_MASS = 1.0f;

        Dictionary<MAVState, Vector3> offsets = new();
        private Dictionary<MAVState, AdaptiveFormationController> controllers = new();
        private Dictionary<MAVState, DateTime> timestamps = new();
        private PointLatLngAlt masterpos = new();
        private DateTime lastUpdate = DateTime.UtcNow;
        private LoadAttitudeController attitudeController = new();
        private TensionSolver tensionSolver = new();
        private float minSeparation = 0.00005f;
        private float avoidanceGain = 0.0001f;

        public void setOffsets(MAVState mav, double x, double y, double z) => offsets[mav] = new Vector3(x, y, z);

        public Vector3 getOffsets(MAVState mav) => offsets.ContainsKey(mav) ? offsets[mav] : new Vector3(offsets.Count, 0, 0);

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
                var f = followers[i];
                pitchLoads[i] = Math.Abs(f.cs.pitchspeed) + Math.Abs(f.cs.rollspeed);
            }

            double total = 0;
            foreach (var l in pitchLoads) total += l;
            if (total == 0) total = 1; // avoid division by zero

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
                    CoordinateTransformationFactory ctfac = new();
                    IGeographicCoordinateSystem wgs84 = GeographicCoordinateSystem.WGS84;
                    ICoordinateTransformation trans = ctfac.CreateFromCoordinateSystems(wgs84, utm);

                    var pll = new[] { Leader.cs.lng, Leader.cs.lat };
                    double[] pLeader;
                    try { pLeader = trans.MathTransform.Transform(pll); }
                    catch { return; }

                    var heading = -Leader.cs.yaw * MathHelper.deg2rad;
                    var dx = offset.x * Math.Cos(heading) - offset.y * Math.Sin(heading);
                    var dy = offset.x * Math.Sin(heading) + offset.y * Math.Cos(heading);

                    pLeader[0] += dx;
                    pLeader[1] += dy;

                    double[] pFollower;
                    try { pFollower = trans.MathTransform.Transform(new[] { mav.cs.lng, mav.cs.lat }); }
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

                            Vector3 rel = new((float)(mav.cs.lat - other.cs.lat), (float)(mav.cs.lng - other.cs.lng), (float)(mav.cs.alt - other.cs.alt));
                            float dist = rel.Length();
                            if (dist < minSeparation && dist > 0.000001f)
                            {
                                float strength = avoidanceGain / (dist * dist);
                                avoidance += rel.Normalize() * strength;
                            }
                        }
                    }

                    if (!timestamps.ContainsKey(mav)) timestamps[mav] = DateTime.UtcNow;
                    float dt = (float)(DateTime.UtcNow - timestamps[mav]).TotalSeconds;
                    timestamps[mav] = DateTime.UtcNow;

                    Vector3 control = controllers[mav].ComputeControl(posError, velError, dt);
                    control += avoidance;
                    control += attitudeController.CompensateRigidBodyDynamics(Leader, mav, offsets);
                    control += tensionSolver.ComputeTensionCorrectionBalanced(Leader, mav, offsets);

                    MAVLink.mavlink_set_attitude_target_t att_target = new();
                    att_target.target_system = mav.sysid;
                    att_target.target_component = mav.compid;
                    att_target.type_mask = 0b00000100;

                    // Convert net vertical acceleration to thrust (add gravity)
                    double verticalThrust = control.z + GRAVITY;
                    att_target.thrust = (float)MathHelper.constrain(verticalThrust / MAX_THRUST_N, 0.1, 1);

                    Quaternion q = Quaternion.from_euler312(control.x * MathHelper.deg2rad, control.y * MathHelper.deg2rad, 0);
                    att_target.q = new float[4] { (float)q.q1, (float)q.q2, (float)q.q3, (float)q.q4 };

                    port.sendPacket(att_target, mav.sysid, mav.compid);
                }
            }
        }
    }
}  
