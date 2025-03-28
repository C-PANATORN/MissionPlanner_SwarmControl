using MissionPlanner.ArduPilot;
using MissionPlanner.Utilities;
using ProjNet.CoordinateSystems;
using ProjNet.CoordinateSystems.Transformations;
using System;
using System.Collections.Generic;
using GeoAPI.CoordinateSystems;
using GeoAPI.CoordinateSystems.Transformations;
using Vector3 = MissionPlanner.Utilities.Vector3;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.RootFinding;

namespace MissionPlanner.Swarm
{
    class Formation : Swarm
    {
        private Dictionary<MAVState, Vector3> offsets = new Dictionary<MAVState, Vector3>();
        private Dictionary<MAVState, Tuple<PID, PID, PID, PID>> pids = new Dictionary<MAVState, Tuple<PID, PID, PID, PID>>();
        private Dictionary<MAVState, Vector3> compFiltered = new Dictionary<MAVState, Vector3>();
        private PointLatLngAlt masterpos = new PointLatLngAlt();
        private LoadAttitudeController payloadController = new LoadAttitudeController();

        private float payloadGain = 0.5f;
        private float maxRollComp = 10f;
        private float maxPitchComp = 10f;
        private float compTau = 0.2f;
        private float blendAlpha = 0.7f;

        CoordinateTransformationFactory ctfac = new CoordinateTransformationFactory();
        IGeographicCoordinateSystem wgs84 = GeographicCoordinateSystem.WGS84;

        public void setOffsets(MAVState mav, double x, double y, double z) => offsets[mav] = new Vector3((float)x, (float)y, (float)z);
        public Vector3 getOffsets(MAVState mav) => offsets.ContainsKey(mav) ? offsets[mav] : new Vector3(offsets.Count, 0, 0);

        public override void Update()
        {
            if (MainV2.comPort.MAV.cs.lat == 0 || MainV2.comPort.MAV.cs.lng == 0) return;
            if (Leader == null) Leader = MainV2.comPort.MAV;
            masterpos = new PointLatLngAlt(Leader.cs.lat, Leader.cs.lng, Leader.cs.alt, "");
        }

        private double wrap_180(double input)
        {
            if (input > 180) return input - 360;
            if (input < -180) return input + 360;
            return input;
        }

        public override void SendCommand()
        {
            if (masterpos.Lat == 0 || masterpos.Lng == 0) return;
            foreach (var port in MainV2.Comports)
            {
                foreach (var mav in port.MAVlist)
                {
                    if (mav == Leader) continue;
                    PointLatLngAlt target = new PointLatLngAlt(masterpos);
                    try
                    {
                        int utmzone = (int)((masterpos.Lng + 180.0) / 6.0);
                        IProjectedCoordinateSystem utm = ProjectedCoordinateSystem.WGS84_UTM(utmzone, masterpos.Lat >= 0);
                        var trans = ctfac.CreateFromCoordinateSystems(wgs84, utm);
                        double[] pLeader = trans.MathTransform.Transform(new double[] { target.Lng, target.Lat });
                        double heading = -Leader.cs.yaw * MathHelper.deg2rad;
                        Vector3 offset = getOffsets(mav);
                        pLeader[0] += offset.x * Math.Cos(heading) - offset.y * Math.Sin(heading);
                        pLeader[1] += offset.x * Math.Sin(heading) + offset.y * Math.Cos(heading);
                        double[] inv = trans.MathTransform.Inverse().Transform(pLeader);
                        target.Lat = inv[1]; target.Lng = inv[0]; target.Alt += offset.z;

                        if (mav.cs.firmware == Firmwares.ArduPlane)
                        {
                            double dist = target.GetDistance(mav.cs.Location);
                            double targyaw = mav.cs.Location.GetBearing(target);
                            double yawerror = wrap_180(targyaw - mav.cs.yaw);
                            Vector3 payloadComp = payloadController.CompensateRigidBodyDynamics(Leader, mav, offsets);
                            float dt = (float)(DateTime.UtcNow - DateTime.MinValue).TotalSeconds;
                            Vector3 prev = compFiltered.ContainsKey(mav) ? compFiltered[mav] : Vector3.Zero;
                            float alpha = dt / (compTau + dt);
                            Vector3 filtered = prev + (payloadComp - prev) * alpha;
                            compFiltered[mav] = filtered;
                            float rollComp = (float)MathHelper.constrain(filtered.x * payloadGain, -maxRollComp, maxRollComp);
                            float pitchComp = (float)MathHelper.constrain(filtered.y * payloadGain, -maxPitchComp, maxPitchComp);
                            float newroll = rollComp * blendAlpha;
                            float newpitch = pitchComp * blendAlpha;
                            Quaternion q = Quaternion.from_euler312(newroll * MathHelper.deg2rad, newpitch * MathHelper.deg2rad, (float)yawerror * MathHelper.deg2rad);
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
                            Vector3 vel = new Vector3(Leader.cs.vx, Leader.cs.vy, Leader.cs.vz);
                            port.setPositionTargetGlobalInt(mav.sysid, mav.compid, true, true, false, false,
                                MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT_INT,
                                target.Lat, target.Lng, target.Alt,
                                vel.x, vel.y, vel.z, 0, 0);
                        }
                    }
                    catch (Exception ex) { Console.WriteLine("SendCommand failed: " + ex); }
                }
            }
        }
    }

    public static class SwarmConstants
    {
        public const float DEFAULT_LEADER_MASS = 1.0f;
    }

    public static class VectorUtils
    {
        public static Vector3 NormalizeVector(Vector3 v)
        {
            double len = Math.Sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
            return len == 0 ? Vector3.Zero : new Vector3((float)(v.x / len), (float)(v.y / len), (float)(v.z / len));
        }
    }

    public class PID
    {
        private float _dt;
        private readonly float M_2PI = (float)(Math.PI * 2);
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
            _integrator = 0;
            _input = 0;
            _derivative = 0;
            _kp = initial_p;
            _ki = initial_i;
            _kd = initial_d;
            _imax = Math.Abs(initial_imax);
            filt_hz(initial_filt_hz);
            _ff = initial_ff;
            _flags._reset_filter = true;
        }
        public void set_dt(float dt) => _dt = dt;
        public void filt_hz(float hz) => _filt_hz = Math.Max(hz, AC_PID_FILT_HZ_MIN);
        public void set_input_filter_all(float input)
        {
            if (!isfinite(input))
                return;
            if (_flags._reset_filter)
            {
                _flags._reset_filter = false;
                _input = input;
                _derivative = 0;
            }
            float change = get_filt_alpha() * (input - _input);
            _input += change;
            if (_dt > 0)
                _derivative = change / _dt;
        }
        private bool isfinite(float x) => !float.IsInfinity(x);
        public float get_p() { _pid_info.P = _input * _kp; return _pid_info.P; }
        public float get_i()
        {

            if (_ki != 0 && _dt != 0)
            {
                _integrator += (_input * _ki) * _dt;
                _integrator = (float)MathHelper.constrain(_integrator, -_imax, _imax);
                _pid_info.I = _integrator;
                return _integrator;
            }
            return 0;

        }
        public float get_d() { _pid_info.D = _kd * _derivative; return _pid_info.D; }
        public float get_pid() => get_p() + get_i() + get_d();
        public void reset_I() => _integrator = 0;
        public float get_filt_alpha() => _filt_hz == 0 ? 1f : _dt / (_dt + 1f / (M_2PI * _filt_hz));
        internal class flags { internal bool _reset_filter; }
        private flags _flags = new flags();
        private pid_info _pid_info = new pid_info();
        internal class pid_info { internal float P, I, D, FF; }
    }
    public class LoadAttitudeController
    {
        public double PayloadMass { get; set; } = SwarmConstants.DEFAULT_LEADER_MASS;

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
            if (!offsets.ContainsKey(follower)) return Vector3.Zero;

            var pts = new List<Vector3>(offsets.Values);
            if (pts.Count < 3) return Vector3.Zero;

            var leaderAcc = new Vector3(leader.cs.ax, leader.cs.ay, leader.cs.az);
            var dynamicOffset = offsets[follower] + leaderAcc * 0.1f;

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
                var qi = VectorUtils.NormalizeVector(dynamicOffset);
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
            double r = roll * MathHelper.deg2rad;
            double p = pitch * MathHelper.deg2rad;
            double y = yaw * MathHelper.deg2rad;

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
            if (angle > 180) return angle - 360;
            if (angle < -180) return angle + 360;
            return angle;
        }
    }
}