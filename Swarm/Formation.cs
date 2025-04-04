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

    // Helper class for math functions.
    internal static class MathHelper
    {
        public const double Deg2Rad = Math.PI / 180.0;
        public const double Rad2Deg = 180.0 / Math.PI;
        public static double Constrain(double value, double min, double max)
        {
            if (value < min)
                return min;
            if (value > max)
                return max;
            return value;
        }
    }

    // Helper class for vector normalization.
    internal static class VectorUtils
    {
        public static Vector3 NormalizeVector(Vector3 v)
        {
            double length = Math.Sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
            return length == 0 ? Vector3.Zero : new Vector3((float)(v.x / length), (float)(v.y / length), (float)(v.z / length));
        }
    }

    internal static class SwarmConstants
    {
        public const float DefaultLeaderMass = 1.0f;
    }

    // Formation class: Implements a cascaded controller that combines feedforward dynamic compensation and PID feedback.
    internal class Formation : Swarm
    {
        private readonly Dictionary<MAVState, Vector3> _offsets = new Dictionary<MAVState, Vector3>();
        private readonly Dictionary<MAVState, Tuple<PID, PID, PID, PID>> _pids = new Dictionary<MAVState, Tuple<PID, PID, PID, PID>>();
        private readonly Dictionary<MAVState, Vector3> _compFiltered = new Dictionary<MAVState, Vector3>();
        private PointLatLngAlt _masterPos = new PointLatLngAlt();
        private readonly LoadAttitudeController _payloadController = new LoadAttitudeController();

        // Feedforward parameters.
        private const float PayloadGain = 0.5f;
        private const float MaxRollComp = 10f;
        private const float MaxPitchComp = 10f;
        private const float CompTau = 0.2f;
        private const float BlendAlpha = 0.7f;

        // Coordinate transformation members.
        private readonly CoordinateTransformationFactory _ctfac = new CoordinateTransformationFactory();
        private readonly IGeographicCoordinateSystem _wgs84 = GeographicCoordinateSystem.WGS84;

        // For proper delta time calculation, store the last update time.
        private DateTime _lastUpdateTime = DateTime.UtcNow;

        public void SetOffsets(MAVState mav, double x, double y, double z) =>
            _offsets[mav] = new Vector3((float)x, (float)y, (float)z);

        public Vector3 GetOffsets(MAVState mav) =>
            _offsets.ContainsKey(mav) ? _offsets[mav] : new Vector3(_offsets.Count, 0, 0);

        public override void Update()
        {
            if (MainV2.comPort.MAV.cs.lat == 0 || MainV2.comPort.MAV.cs.lng == 0)
                return;
            if (Leader == null)
                Leader = MainV2.comPort.MAV;
            _masterPos = new PointLatLngAlt(Leader.cs.lat, Leader.cs.lng, Leader.cs.alt, "");

            // Update dt for filtering by storing the current update time.
            _lastUpdateTime = DateTime.UtcNow;
        }

        // Utility method to wrap an angle into the [-180, 180] range.
        private double Wrap180(double angle)
        {
            if (angle > 180)
                return angle - 360;
            if (angle < -180)
                return angle + 360;
            return angle;
        }

        public override void SendCommand()
        {
            if (_masterPos.Lat == 0 || _masterPos.Lng == 0)
                return;

            foreach (var port in MainV2.Comports)
            {
                foreach (var mav in port.MAVlist)
                {
                    if (mav == Leader)
                        continue;

                    PointLatLngAlt target = new PointLatLngAlt(_masterPos);
                    try
                    {
                        // Transform leader's position to UTM and apply follower's position offset.
                        int utmZone = (int)((_masterPos.Lng + 180.0) / 6.0);
                        IProjectedCoordinateSystem utm = ProjectedCoordinateSystem.WGS84_UTM(utmZone, _masterPos.Lat >= 0);
                        var trans = _ctfac.CreateFromCoordinateSystems(_wgs84, utm);
                        double[] pLeader = trans.MathTransform.Transform(new double[] { target.Lng, target.Lat });
                        double heading = -Leader.cs.yaw * MathHelper.Deg2Rad;
                        Vector3 offset = GetOffsets(mav);
                        pLeader[0] += offset.x * Math.Cos(heading) - offset.y * Math.Sin(heading);
                        pLeader[1] += offset.x * Math.Sin(heading) + offset.y * Math.Cos(heading);
                        double[] inv = trans.MathTransform.Inverse().Transform(pLeader);
                        target.Lat = inv[1];
                        target.Lng = inv[0];
                        target.Alt += offset.z;

                        if (mav.cs.firmware == Firmwares.ArduPlane)
                        {
                            // Compute yaw error based on target bearing.
                            double targYaw = mav.cs.Location.GetBearing(target);
                            double yawError = Wrap180(targYaw - mav.cs.yaw);

                            // Use leader's current attitude as desired attitude.
                            double desiredRoll = Leader.cs.roll;
                            double desiredPitch = Leader.cs.pitch;
                            double errorRoll = desiredRoll - mav.cs.roll;
                            double errorPitch = desiredPitch - mav.cs.pitch;

                            // Compute feedforward payload compensation.
                            Vector3 payloadComp = _payloadController.CompensateRigidBodyDynamics(Leader, mav, _offsets);

                            // Compute dt as the elapsed time since last update.
                            // (For a production system, store and update _lastUpdateTime properly.)
                            float dt = (float)(DateTime.UtcNow - _lastUpdateTime).TotalSeconds;
                            // If dt is very small (or zero), you may wish to default it to a nominal value.
                            if (dt <= 0)
                                dt = 0.1f;

                            Vector3 prevFiltered = _compFiltered.ContainsKey(mav) ? _compFiltered[mav] : Vector3.Zero;
                            float alpha = dt / (CompTau + dt);
                            Vector3 filteredComp = prevFiltered + (payloadComp - prevFiltered) * alpha;
                            _compFiltered[mav] = filteredComp;
                            float ffRoll = (float)MathHelper.Constrain(filteredComp.x * PayloadGain, -MaxRollComp, MaxRollComp);
                            float ffPitch = (float)MathHelper.Constrain(filteredComp.y * PayloadGain, -MaxPitchComp, MaxPitchComp);

                            // Compute PID feedback corrections based on the computed attitude errors.
                            if (!_pids.TryGetValue(mav, out Tuple<PID, PID, PID, PID> pidTuple))
                            {
                                pidTuple = new Tuple<PID, PID, PID, PID>(
                                    new PID(1f, 0.03f, 0.02f, 10, 20, 0.1f, 0),  // roll PID
                                    new PID(1f, 0.03f, 0.02f, 10, 20, 0.1f, 0),  // pitch PID
                                    new PID(1f, 0.0f, 0.0f, 15, 20, 0.1f, 0),     // yaw PID (if needed)
                                    new PID(0.01f, 0.001f, 0, 0.5f, 20, 0.1f, 0)   // thrust PID (if used)
                                );
                                _pids[mav] = pidTuple;
                            }
                            PID rollPID = pidTuple.Item1;
                            PID pitchPID = pidTuple.Item2;
                            rollPID.SetInputFilterAll((float)errorRoll);
                            pitchPID.SetInputFilterAll((float)errorPitch);
                            double fbRoll = rollPID.GetPID();
                            double fbPitch = pitchPID.GetPID();

                            // Combine the feedback (PID) and feedforward compensation terms.
                            double finalRoll = fbRoll + (ffRoll * BlendAlpha);
                            double finalPitch = fbPitch + (ffPitch * BlendAlpha);

                            // Form the final attitude command as a quaternion.
                            Quaternion q = Quaternion.from_euler312(finalRoll * MathHelper.Deg2Rad,
                                                                      finalPitch * MathHelper.Deg2Rad,
                                                                      (float)yawError * MathHelper.Deg2Rad);
                            var att = new MAVLink.mavlink_set_attitude_target_t
                            {
                                target_system = mav.sysid,
                                target_component = mav.compid,
                                type_mask = 0b10000101,
                                q = new float[] { (float)q.q1, (float)q.q2, (float)q.q3, (float)q.q4 }
                            };
                            port.sendPacket(att, mav.sysid, mav.compid);
                        }
                        else
                        {
                            // For non-ArduPlane firmware, issue a position target command.
                            Vector3 vel = new Vector3(Leader.cs.vx, Leader.cs.vy, Leader.cs.vz);
                            port.setPositionTargetGlobalInt(mav.sysid, mav.compid, true, true, false, false,
                                MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT_INT,
                                target.Lat, target.Lng, target.Alt,
                                vel.x, vel.y, vel.z, 0, 0);
                        }
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine("SendCommand failed: " + ex);
                    }
                }
            }
        }
    }

    // --- PID Controller Implementation ---
    internal class PID
    {
        private float _dt;
        private readonly float _m2Pi = (float)(Math.PI * 2);
        private float _input;
        private float _derivative;
        private float _kp;
        private float _ki;
        private float _integrator;
        private float _imax;
        private float _kd;
        private float _ff;
        private float _filtHz = AcPidFiltHzDefault;
        private const float AcPidFiltHzDefault = 20.0f;
        private const float AcPidFiltHzMin = 0.01f;

        public PID(float initialP, float initialI, float initialD, float initialIMax, float initialFiltHz, float dt, float initialFF)
        {
            _dt = dt;
            _integrator = 0;
            _input = 0;
            _derivative = 0;
            _kp = initialP;
            _ki = initialI;
            _kd = initialD;
            _imax = Math.Abs(initialIMax);
            FiltHz(initialFiltHz);
            _ff = initialFF;
            _flags._resetFilter = true;
        }

        public void SetDt(float dt) => _dt = dt;
        public void FiltHz(float hz) => _filtHz = Math.Max(hz, AcPidFiltHzMin);
        public void SetInputFilterAll(float input)
        {
            if (!IsFinite(input))
                return;
            if (_flags._resetFilter)
            {
                _flags._resetFilter = false;
                _input = input;
                _derivative = 0;
            }
            float change = GetFiltAlpha() * (input - _input);
            _input += change;
            if (_dt > 0)
                _derivative = change / _dt;
        }
        private bool IsFinite(float x) => !float.IsInfinity(x);
        public float GetP() { _pidInfo.P = _input * _kp; return _pidInfo.P; }
        public float GetI()
        {
            if (_ki != 0 && _dt != 0)
            {
                _integrator += (_input * _ki) * _dt;
                _integrator = (float)MathHelper.Constrain(_integrator, -_imax, _imax);
                _pidInfo.I = _integrator;
                return _integrator;
            }
            return 0;
        }
        public float GetD() { _pidInfo.D = _kd * _derivative; return _pidInfo.D; }
        public float GetPID() => GetP() + GetI() + GetD();
        public void ResetI() => _integrator = 0;
        public float GetFiltAlpha() => _filtHz == 0 ? 1f : _dt / (_dt + 1f / (_m2Pi * _filtHz));
        internal class Flags { internal bool _resetFilter; }
        private Flags _flags = new Flags();
        private PIDInfo _pidInfo = new PIDInfo();
        internal class PIDInfo { internal float P, I, D, FF; }
    }

    // --- LoadAttitudeController Implementation ---
    internal class LoadAttitudeController
    {
        public double PayloadMass { get; set; } = SwarmConstants.DefaultLeaderMass;

        // Payload inertia matrix JL (kg·m²)
        private static readonly Matrix<double> PayloadInertia = Matrix<double>.Build.DenseOfArray(new double[,]
        {
            { 0.021, 0, 0 },
            { 0, 0.0187, 0 },
            { 0, 0, 0.0397 }
        });

        private static readonly Dictionary<int, Tuple<float, float>> YawHistory = new Dictionary<int, Tuple<float, float>>();

        public Vector3 CompensateRigidBodyDynamics(MAVState leader, MAVState follower, Dictionary<MAVState, Vector3> offsets)
        {
            if (!offsets.ContainsKey(follower))
                return Vector3.Zero;

            var pts = new List<Vector3>(offsets.Values);
            if (pts.Count < 3)
                return Vector3.Zero;

            var staticOffset = offsets[follower];

            float yawRate = EstimateYawRate(leader);
            float compTau = Math.Abs(yawRate) > 10 ? 0.05f : 0.2f;
            float dt = 0.05f;
            float alpha = dt / (compTau + dt);

            float m = (float)PayloadMass;
            var accel = Vector<double>.Build.DenseOfArray(new double[] { leader.cs.ax * m, leader.cs.ay * m, (leader.cs.az + 9.81f) * m });
            var grav = Vector<double>.Build.DenseOfArray(new double[] { 0, 0, -9.81 }) * m;
            var R = BuildRotationMatrix(leader.cs.roll, leader.cs.pitch, leader.cs.yaw);
            var Wf = -R.Transpose() * (accel + grav);

            var omega = Vector<double>.Build.Dense(3);
            var omegaDot = Vector<double>.Build.Dense(3);
            var Wm = -(PayloadInertia * omegaDot + Cross(omega, PayloadInertia * omega));

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

            var T = Phi.PseudoInverse() * W;
            int idx = new List<MAVState>(offsets.Keys).IndexOf(follower);
            if (idx >= 0 && idx < T.Count)
            {
                var qi = VectorUtils.NormalizeVector(staticOffset);
                return qi * (float)Math.Max(0, T[idx]) * alpha;
            }

            return Vector3.Zero;
        }

        private float EstimateYawRate(MAVState leader)
        {
            int id = leader.sysid;
            float currentYaw = leader.cs.yaw;
            float currentTime = (float)(DateTime.UtcNow - new DateTime(1970, 1, 1)).TotalSeconds;

            if (YawHistory.ContainsKey(id))
            {
                var entry = YawHistory[id];
                float dyaw = Wrap180(currentYaw - entry.Item1);
                float dt = currentTime - entry.Item2;
                YawHistory[id] = Tuple.Create(currentYaw, currentTime);
                return dt > 0.001f ? dyaw / dt : 0f;
            }

            YawHistory[id] = Tuple.Create(currentYaw, currentTime);
            return 0f;
        }

        private Matrix<double> BuildRotationMatrix(float roll, float pitch, float yaw)
        {
            double r = roll * MathHelper.Deg2Rad;
            double p = pitch * MathHelper.Deg2Rad;
            double y = yaw * MathHelper.Deg2Rad;

            double cr = Math.Cos(r), sr = Math.Sin(r);
            double cp = Math.Cos(p), sp = Math.Sin(p);
            double cy = Math.Cos(y), sy = Math.Sin(y);

            return Matrix<double>.Build.DenseOfArray(new double[,]
            {
                { cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr },
                { sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr },
                { -sp, cp * sr, cp * cr }
            });
        }

        private Vector<double> Cross(Vector<double> a, Vector<double> b)
        {
            return Vector<double>.Build.DenseOfArray(new double[]
            {
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]
            });
        }

        private float Wrap180(float angle)
        {
            if (angle > 180)
                return angle - 360;
            if (angle < -180)
                return angle + 360;
            return angle;
        }
    }
}
