using MissionPlanner.ArduPilot;
using MissionPlanner.Utilities;
using ProjNet.CoordinateSystems;
using ProjNet.CoordinateSystems.Transformations;
using System;
using System.Collections.Generic;
using GeoAPI.CoordinateSystems;
using GeoAPI.CoordinateSystems.Transformations;
using Vector3 = MissionPlanner.Utilities.Vector3;

namespace MissionPlanner.Swarm
{
    /// <summary>
    /// Formation control using a leader and two followers that snap into
    /// an equilateral triangle formation (4 m side length) without intersecting paths.
    /// </summary>
    class Formation : Swarm
    {
        // Dictionary for PID controllers per MAV.
        private Dictionary<MAVState, Tuple<PID, PID, PID, PID>> pids =
            new Dictionary<MAVState, Tuple<PID, PID, PID, PID>>();

        // Latest known leader position.
        private PointLatLngAlt masterpos = new PointLatLngAlt();

        // Coordinate transformation helpers.
        CoordinateTransformationFactory ctfac = new CoordinateTransformationFactory();
        IGeographicCoordinateSystem wgs84 = GeographicCoordinateSystem.WGS84;

        public override void Update()
        {
            if (MainV2.comPort.MAV.cs.lat == 0 || MainV2.comPort.MAV.cs.lng == 0)
                return;

            if (Leader == null)
                Leader = MainV2.comPort.MAV;

            masterpos = new PointLatLngAlt(Leader.cs.lat, Leader.cs.lng, Leader.cs.alt, "");
        }

        double wrap_180(double input)
        {
            if (input > 180)
                return input - 360;
            if (input < -180)
                return input + 360;
            return input;
        }

        /// <summary>
        /// Send command for formation control.
        /// For exactly two followers, we use fixed formation offsets so that:
        /// - The leader is the front vertex,
        /// - Followers are positioned at the base of the equilateral triangle (side = 4 m).
        /// In the leader’s local frame (x: forward, y: right), the two offsets are:
        /// Left follower: (-3.464, -2, 0)
        /// Right follower: (-3.464,  2, 0)
        /// </summary>
        public override void SendCommand()
        {
            if (masterpos.Lat == 0 || masterpos.Lng == 0)
                return;

            // Counter for assigning fixed formation positions to followers.
            int followerIndex = 0;

            foreach (var port in MainV2.Comports.ToArray())
            {
                foreach (var mav in port.MAVlist)
                {
                    if (mav == Leader)
                        continue;

                    // Compute formation offset regardless of any previous settings.
                    // In the leader's body frame:
                    //   x: forward (negative means behind the leader)
                    //   y: lateral (negative is left if leader's right is positive)
                    Vector3 formationOffset;
                    if (followerIndex == 0)
                    {
                        // Left follower offset: behind by 3.464 m and 2 m to the left.
                        formationOffset = new Vector3(-3.464, -2.0, 0);
                    }
                    else if (followerIndex == 1)
                    {
                        // Right follower offset: behind by 3.464 m and 2 m to the right.
                        formationOffset = new Vector3(-3.464, 2.0, 0);
                    }
                    else
                    {
                        // Fallback: if more than two followers, assign a default offset.
                        formationOffset = new Vector3(-3.464, 0, 0);
                    }
                    followerIndex++;

                    // Start with leader's position as target.
                    PointLatLngAlt target = new PointLatLngAlt(masterpos);

                    try
                    {
                        // Convert leader position (WGS84) to UTM.
                        int utmzone = (int)((masterpos.Lng - -186.0) / 6.0);
                        IProjectedCoordinateSystem utm = ProjectedCoordinateSystem.WGS84_UTM(
                            utmzone, masterpos.Lat < 0 ? false : true);
                        ICoordinateTransformation trans = ctfac.CreateFromCoordinateSystems(wgs84, utm);

                        double[] pll1 = { target.Lng, target.Lat };
                        double[] p1 = trans.MathTransform.Transform(pll1);

                        // Calculate heading (note: heading is the negative of leader yaw).
                        double heading = -Leader.cs.yaw;

                        // Apply formation offset in UTM space.
                        p1[0] += formationOffset.x * Math.Cos(heading * MathHelper.deg2rad) -
                                 formationOffset.y * Math.Sin(heading * MathHelper.deg2rad);
                        p1[1] += formationOffset.x * Math.Sin(heading * MathHelper.deg2rad) +
                                 formationOffset.y * Math.Cos(heading * MathHelper.deg2rad);

                        // Convert back from UTM to WGS84.
                        IMathTransform inverseTransform = trans.MathTransform.Inverse();
                        double[] point = inverseTransform.Transform(p1);
                        target.Lat = point[1];
                        target.Lng = point[0];
                        target.Alt += formationOffset.z;

                        // --- Send commands based on firmware type ---
                        if (mav.cs.firmware == Firmwares.ArduPlane)
                        {
                            // Calculate distance and bearing to target.
                            var dist = target.GetDistance(mav.cs.Location);
                            var targyaw = mav.cs.Location.GetBearing(target);
                            // Create trailer and leader target positions to avoid intersecting trajectories.
                            var targettrailer = target.newpos(Leader.cs.yaw, Math.Abs(dist) * -0.25);
                            var targetleader = target.newpos(Leader.cs.yaw, 10 + dist);
                            var yawerror = wrap_180(targyaw - mav.cs.yaw);

                            if (dist < 100)
                            {
                                targyaw = mav.cs.Location.GetBearing(targetleader);
                                yawerror = wrap_180(targyaw - mav.cs.yaw);
                                var targBearing = mav.cs.Location.GetBearing(target);
                                if (Math.Abs(wrap_180(targBearing - targyaw)) > 45)
                                    dist *= -1;
                            }
                            else
                            {
                                targyaw = mav.cs.Location.GetBearing(targettrailer);
                                yawerror = wrap_180(targyaw - mav.cs.yaw);
                            }

                            // Update guided mode position.
                            mav.GuidedMode.x = (int)(target.Lat * 1e7);
                            mav.GuidedMode.y = (int)(target.Lng * 1e7);
                            mav.GuidedMode.z = (float)target.Alt;

                            MAVLink.mavlink_set_attitude_target_t att_target = new MAVLink.mavlink_set_attitude_target_t();
                            att_target.target_system = mav.sysid;
                            att_target.target_component = mav.compid;
                            att_target.type_mask = 0xff;

                            Tuple<PID, PID, PID, PID> pid;
                            if (pids.ContainsKey(mav))
                            {
                                pid = pids[mav];
                            }
                            else
                            {
                                pid = new Tuple<PID, PID, PID, PID>(
                                    new PID(1f, 0.03f, 0.02f, 10, 20, 0.1f, 0),
                                    new PID(1f, 0.03f, 0.02f, 10, 20, 0.1f, 0),
                                    new PID(1f, 0f, 0.00f, 15, 20, 0.1f, 0),
                                    new PID(0.01f, 0.001f, 0f, 0.5f, 20, 0.1f, 0));
                                pids.Add(mav, pid);
                            }

                            var rollp = pid.Item1;
                            var pitchp = pid.Item2;
                            var yawp = pid.Item3;
                            var thrustp = pid.Item4;
                            double newroll = 0d;
                            double newpitch = 0d;

                            // Altitude control.
                            var altdelta = target.Alt - mav.cs.alt;
                            newpitch = altdelta;
                            att_target.type_mask -= 0b00000010;
                            pitchp.set_input_filter_all((float)altdelta);
                            newpitch = pitchp.get_pid();

                            // Roll control.
                            var leaderturnrad = CurrentState.fromDistDisplayUnit(Leader.cs.radius);
                            var mavturnradius = leaderturnrad;
                            var distToTarget = mav.cs.Location.GetDistance(target);
                            var bearingToTarget = mav.cs.Location.GetBearing(target);
                            if (distToTarget < 30)
                                bearingToTarget = mav.cs.Location.GetBearing(targetleader);
                            if (distToTarget > 100)
                                bearingToTarget = mav.cs.Location.GetBearing(targettrailer);
                            var bearingDelta = wrap_180(bearingToTarget - mav.cs.yaw);
                            double tangent90 = bearingDelta > 0 ? 90 : -90;
                            if (Math.Abs(bearingDelta) < 85)
                            {
                                double insideAngle = Math.Abs(tangent90 - bearingDelta);
                                double angleCenter = 180 - insideAngle * 2;
                                double sine1 = Math.Max(distToTarget, 40) / Math.Sin(angleCenter * MathHelper.deg2rad);
                                double radius = sine1 * Math.Sin(insideAngle * MathHelper.deg2rad);
                                radius = (Math.Abs(radius) + Math.Abs(mavturnradius)) / 2;
                                double angleBank = ((mav.cs.groundspeed * mav.cs.groundspeed) / radius) / 9.8;
                                angleBank *= MathHelper.rad2deg;
                                newroll = (bearingDelta > 0) ? Math.Abs(angleBank) : -Math.Abs(angleBank);
                            }
                            newroll += MathHelper.constrain(bearingDelta, -20, 20);

                            // Thrust control.
                            att_target.type_mask -= 0b01000000;
                            thrustp.set_input_filter_all((float)distToTarget);
                            if (distToTarget > 40)
                                thrustp.reset_I();
                            att_target.thrust = (float)MathHelper.constrain(thrustp.get_pid(), 0.1, 1);

                            Quaternion q = Quaternion.from_euler312(newroll * MathHelper.deg2rad,
                                newpitch * MathHelper.deg2rad, yawerror * MathHelper.deg2rad);
                            att_target.q = new float[4];
                            att_target.q[0] = (float)q.q1;
                            att_target.q[1] = (float)q.q2;
                            att_target.q[2] = (float)q.q3;
                            att_target.q[3] = (float)q.q4;
                            att_target.type_mask -= 0b10000101;

                            Console.WriteLine("sysid {0} - {1} dist {2} r {3} p {4} y {5}",
                                mav.sysid, att_target.thrust, distToTarget, newroll, newpitch, (targyaw - mav.cs.yaw));

                            port.sendPacket(att_target, mav.sysid, mav.compid);
                        }
                        else
                        {
                            // For firmwares other than ArduPlane, set the position target and yaw.
                            Vector3 vel = new Vector3(Leader.cs.vx, Leader.cs.vy, Leader.cs.vz);
                            port.setPositionTargetGlobalInt(mav.sysid, mav.compid, true, true, false, false,
                                MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT_INT, target.Lat, target.Lng, target.Alt,
                                vel.x, vel.y, vel.z, 0, 0);

                            if (!gimbal)
                            {
                                if (Math.Abs(mav.cs.yaw - Leader.cs.yaw) > 3)
                                    port.doCommand(mav.sysid, mav.compid, MAVLink.MAV_CMD.CONDITION_YAW,
                                        Leader.cs.yaw, 100.0f, 0, 0, 0, 0, 0, false);
                            }
                            else
                            {
                                if (Math.Abs(mav.cs.yaw - Leader.cs.yaw) > 3)
                                    port.setMountControl(mav.sysid, mav.compid, 45, 0, Leader.cs.yaw, false);
                            }
                        }
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine("Failed to send command for " + mav.ToString() + "\n" + ex.ToString());
                    }
                }
            }
        }

        public bool gimbal { get; set; }
    }

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
            {
                return;
            }
            if (_flags._reset_filter)
            {
                _flags._reset_filter = false;
                _input = input;
                _derivative = 0.0f;
            }
            float input_filt_change = get_filt_alpha() * (input - _input);
            _input = _input + input_filt_change;
            if (_dt > 0.0f)
            {
                _derivative = input_filt_change / _dt;
            }
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
                _integrator += ((float)_input * _ki) * _dt;
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
            {
                return 1.0f;
            }
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
}
