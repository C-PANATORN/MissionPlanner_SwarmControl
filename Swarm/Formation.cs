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
        Dictionary<MAVState, Vector3> offsets = new();
        private Dictionary<MAVState, AdaptiveFormationController> controllers = new();
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

        public override void SendCommand()
        {
            if (masterpos.Lat == 0 || masterpos.Lng == 0)
                return;

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
                    var pLeader = trans.MathTransform.Transform(pll);
                    var heading = -Leader.cs.yaw * MathHelper.deg2rad;
                    var dx = offset.x * Math.Cos(heading) - offset.y * Math.Sin(heading);
                    var dy = offset.x * Math.Sin(heading) + offset.y * Math.Cos(heading);

                    pLeader[0] += dx;
                    pLeader[1] += dy;

                    var point = trans.MathTransform.Inverse().Transform(pLeader);
                    desiredPosition = new Vector3((float)point[1], (float)point[0], (float)(Leader.cs.alt + offset.z));
                    desiredVelocity = new Vector3((float)Leader.cs.vx, (float)Leader.cs.vy, (float)Leader.cs.vz);

                    Vector3 posError = desiredPosition - new Vector3((float)mav.cs.lat, (float)mav.cs.lng, (float)mav.cs.alt);
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

                    float dt = (float)(DateTime.UtcNow - lastUpdate).TotalSeconds;
                    lastUpdate = DateTime.UtcNow;

                    Vector3 control = controllers[mav].ComputeControl(posError, velError, dt);
                    control += avoidance;
                    control += attitudeController.CompensateRigidBodyDynamics(Leader, mav);
                    control += tensionSolver.ComputeTensionCorrectionBalanced(Leader, mav, offsets);

                    MAVLink.mavlink_set_attitude_target_t att_target = new();
                    att_target.target_system = mav.sysid;
                    att_target.target_component = mav.compid;
                    att_target.type_mask = 0b00000100;

                    double thrust = control.z + 9.8;
                    att_target.thrust = (float)MathHelper.constrain(thrust / 20.0, 0.1, 1);

                    Quaternion q = Quaternion.from_euler312(control.x * MathHelper.deg2rad, control.y * MathHelper.deg2rad, 0);
                    att_target.q = new float[4] { (float)q.q1, (float)q.q2, (float)q.q3, (float)q.q4 };

                    port.sendPacket(att_target, mav.sysid, mav.compid);
                }
            }
        }
    }

    public class TensionSolver
    {
        public Vector3 ComputeTensionCorrectionBalanced(MAVState leader, MAVState follower, Dictionary<MAVState, Vector3> offsets)
        {
            var W = Vector<double>.Build.DenseOfArray(new[] {
                leader.cs.ax,
                leader.cs.ay,
                leader.cs.az + 9.8f,
                leader.cs.rollspeed,
                leader.cs.pitchspeed,
                leader.cs.yawspeed
            });

            var attachmentPoints = new List<Vector3>(offsets.Values);
            int n = attachmentPoints.Count;
            var Phi = Matrix<double>.Build.Dense(6, n);

            for (int i = 0; i < n; i++)
            {
                var r = attachmentPoints[i];
                var qi = Vector<double>.Build.DenseOfArray(new[] { r.x, r.y, r.z }).Normalize(2);
                Phi[0, i] = qi[0]; Phi[1, i] = qi[1]; Phi[2, i] = qi[2];
                Phi[3, i] = r.y * qi[2] - r.z * qi[1];
                Phi[4, i] = r.z * qi[0] - r.x * qi[2];
                Phi[5, i] = r.x * qi[1] - r.y * qi[0];
            }

            var PhiT = Phi.Transpose();
            var PhiPlus = (PhiT * Phi).Inverse() * PhiT;
            var N = Phi.NullSpace();
            var Lambda = Vector<double>.Build.Dense(N.ColumnCount, 0.1);
            var T = PhiPlus * W + N * Lambda;
            for (int i = 0; i < T.Count; i++)
                T[i] = Math.Max(T[i], 0);

            int followerIndex = new List<MAVState>(offsets.Keys).IndexOf(follower);
            if (followerIndex >= 0 && followerIndex < T.Count)
            {
                var qi = Vector3.Normalize(offsets[follower]);
                return qi * (float)T[followerIndex];
            }

            return Vector3.Zero;
        }
    }
} 
