using System;
using System.Collections.Generic;
using MissionPlanner.ArduPilot;
using MissionPlanner.Utilities;
using ProjNet.CoordinateSystems;
using ProjNet.CoordinateSystems.Transformations;
using GeoAPI.CoordinateSystems;
using GeoAPI.CoordinateSystems.Transformations;
using Vector3 = MissionPlanner.Utilities.Vector3;

namespace MissionPlanner.Swarm
{
    // --- Extension methods for Vector3 ---
    public static class Vector3Extensions
    {
        public static float Length(this Vector3 v)
        {
            return (float)Math.Sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
        }

        public static Vector3 Normalized(this Vector3 v)
        {
            float len = v.Length();
            if (len < 1e-6f) return new Vector3(0, 0, 0);
            return new Vector3(v.x / len, v.y / len, v.z / len);
        }
    }

    /// <summary>Formation control logic (follower commands)</summary>
    internal class Formation : Swarm
    {
        private readonly Dictionary<MAVState, Vector3> offsets = new Dictionary<MAVState, Vector3>();

        // --- Plane PID controllers (original logic) ---
        private readonly Dictionary<MAVState, Tuple<PID, PID, PID, PID>> pids =
            new Dictionary<MAVState, Tuple<PID, PID, PID, PID>>();

        // --- UTM transform helpers ---
        private readonly CoordinateTransformationFactory ctfac = new CoordinateTransformationFactory();
        private readonly IGeographicCoordinateSystem wgs84 = GeographicCoordinateSystem.WGS84;

        private PointLatLngAlt masterpos;
        private DateTime lastUpdateTime = DateTime.UtcNow;

        // ===== Constants for turns / barriers / collision =====
        private const float DEG2RAD = (float)(Math.PI / 180.0);

        // Lead target ahead in turns (previews heading)
        private const float TurnLeadTime = 0.5f;        // s

        // Blend into on-circle target when turning
        private const float TurnBlendStartRate = 8f;    // deg/s
        private const float TurnBlendMaxRate = 45f;   // deg/s

        // Dilate desired radius on the outside of the turn
        private const float OuterDilatePerDeg = 0.011f; // m per |deg/s| * |offset|

        // Radial barrier to avoid cutting inside when exiting a turn
        private const float RadialMinMargin = 1.0f;     // m
        private const float RadialMinScale = 0.80f;    // rMin >= 0.8*|offsetXY|

        // Simple 1/dist^2 repulsive collision avoidance
        private const float dSafeBase = 2.0f;   // m
        private const float RepulsiveBaseGain = 1.3f;   // scale of 1/dist^2
        private const float RepulsiveScale = 1.0f;   // overall scale

        // ==== Hover detection (align followers' yaw with leader when hover) ====
        private const float HoverSpeedXYThresh = 0.25f; // m/s
        private const float HoverVzThresh = 0.25f; // m/s
        private const float HoverYawRateThreshDeg = 3f;    // deg/s

        // --- Minimal yaw rate estimator ---
        private readonly Dictionary<int, Tuple<float, float>> yawHistory = new Dictionary<int, Tuple<float, float>>();

        public void setOffsets(MAVState mav, double x, double y, double z) =>
            offsets[mav] = new Vector3((float)x, (float)y, (float)z);
        public Vector3 getOffsets(MAVState mav) => offsets.ContainsKey(mav) ? offsets[mav] : Vector3.Zero;

        public override void Update()
        {
            var mav0 = MainV2.comPort.MAV;
            if (mav0.cs.lat == 0 || mav0.cs.lng == 0) return;
            if (Leader == null) Leader = mav0;
            masterpos = new PointLatLngAlt(Leader.cs.lat, Leader.cs.lng, Leader.cs.alt, "");
            lastUpdateTime = DateTime.UtcNow;
        }

        private double wrap_180(double a) => a > 180 ? a - 360 : a < -180 ? a + 360 : a;

        private float EstimateYawRateDeg(MAVState m)
        {
            int id = m.sysid;
            float cy = m.cs.yaw; // degrees
            float ct = (float)(DateTime.UtcNow - new DateTime(1970, 1, 1)).TotalSeconds;
            if (yawHistory.ContainsKey(id))
            {
                var prev = yawHistory[id];
                float py = prev.Item1;
                float pt = prev.Item2;
                float dy = (float)wrap_180(cy - py);
                float dt = Math.Max(1e-3f, ct - pt);
                yawHistory[id] = Tuple.Create(cy, ct);
                return dy / dt;
            }
            yawHistory[id] = Tuple.Create(cy, ct);
            return 0f;
        }

        public override void SendCommand()
        {
            if (Leader == null || masterpos.Lat == 0) return;

            // Build UTM transform once per tick
            int zone = (int)((masterpos.Lng + 180.0) / 6.0);
            var utm = ProjectedCoordinateSystem.WGS84_UTM(zone, masterpos.Lat >= 0);
            var trans = ctfac.CreateFromCoordinateSystems(wgs84, utm);
            double[] lXY = trans.MathTransform.Transform(new[] { masterpos.Lng, masterpos.Lat });
            Vector3 leaderUTM = new Vector3((float)lXY[0], (float)lXY[1], (float)masterpos.Alt);

            // Leader motion state, hover detection
            float leaderVxy = (float)Math.Sqrt(Leader.cs.vx * Leader.cs.vx + Leader.cs.vy * Leader.cs.vy);
            float leaderYawRateDeg = EstimateYawRateDeg(Leader);
            bool leaderHover =
                leaderVxy < HoverSpeedXYThresh &&
                Math.Abs(Leader.cs.vz) < HoverVzThresh &&
                Math.Abs(leaderYawRateDeg) < HoverYawRateThreshDeg;

            float leaderYawRad = (float)(Leader.cs.yaw * Math.PI / 180.0);
            float yawAbs = Math.Abs(leaderYawRateDeg);
            float yawRateRad = Math.Max(1e-3f, yawAbs * DEG2RAD);
            float leadSpeed = Math.Max(0.01f, leaderVxy);
            float Rturn = leadSpeed / yawRateRad; // m

            // World-frame forward/left from leader yaw (Mission Planner uses NED yaw degrees)
            float yawRadHeading = (float)(-Leader.cs.yaw * Math.PI / 180.0);
            var fwd = new Vector3((float)Math.Cos(yawRadHeading), (float)Math.Sin(yawRadHeading), 0f);
            var left = new Vector3(-fwd.y, fwd.x, 0f);
            var nrm = (leaderYawRateDeg >= 0) ? left : new Vector3(-left.x, -left.y, 0f);
            var ICC = new Vector3(leaderUTM.x + nrm.x * Rturn, leaderUTM.y + nrm.y * Rturn, leaderUTM.z);

            // Cache current positions of all vehicles (UTM)
            Dictionary<MAVState, Vector3> currentPositions = new Dictionary<MAVState, Vector3>();
            foreach (var port0 in MainV2.Comports)
            {
                foreach (var mav0 in port0.MAVlist)
                {
                    double[] cXY = trans.MathTransform.Transform(new[] { mav0.cs.lng, mav0.cs.lat });
                    currentPositions[mav0] = new Vector3((float)cXY[0], (float)cXY[1], mav0.cs.alt);
                }
            }

            // Command followers
            foreach (var port in MainV2.Comports)
            {
                foreach (var mav in port.MAVlist)
                {
                    if (mav == Leader) continue;

                    // --- Target from rotated offset with heading preview ---
                    Vector3 off = getOffsets(mav);
                    float yawPreview = yawRadHeading + leaderYawRateDeg * DEG2RAD * TurnLeadTime;

                    Vector3 targetUTM = new Vector3(
                        leaderUTM.x + off.x * (float)Math.Cos(yawPreview) - off.y * (float)Math.Sin(yawPreview),
                        leaderUTM.y + off.x * (float)Math.Sin(yawPreview) + off.y * (float)Math.Cos(yawPreview),
                        (float)masterpos.Alt + off.z
                    );

                    // --- On-circle blending when turning ---
                    if (yawAbs >= TurnBlendStartRate && leadSpeed > 0.5f)
                    {
                        Vector3 vecL = new Vector3(leaderUTM.x - ICC.x, leaderUTM.y - ICC.y, 0f);
                        Vector3 vecOffWorld = new Vector3(targetUTM.x - leaderUTM.x, targetUTM.y - leaderUTM.y, 0f);
                        Vector3 vecF = new Vector3(vecL.x + vecOffWorld.x, vecL.y + vecOffWorld.y, 0f);

                        float rF = Math.Max(0.1f, vecF.Length());

                        float extraR = off.Length() * OuterDilatePerDeg * yawAbs; // extra radius for outer lanes
                        float rDesired = Rturn + extraR;

                        Vector3 vecF_on = vecF.Normalized() * rDesired;
                        Vector3 onCircle = new Vector3(ICC.x + vecF_on.x, ICC.y + vecF_on.y, targetUTM.z);

                        float blend = Math.Max(0f, Math.Min(1f, (yawAbs - TurnBlendStartRate) / (TurnBlendMaxRate - TurnBlendStartRate)));
                        targetUTM = new Vector3(
                            targetUTM.x * (1 - blend) + onCircle.x * blend,
                            targetUTM.y * (1 - blend) + onCircle.y * blend,
                            targetUTM.z * (1 - blend) + onCircle.z * blend
                        );
                    }

                    // --- Radial barrier (avoid cutting inside) ---
                    float offXY = new Vector3(off.x, off.y, 0f).Length();
                    float rDesFromLeader = new Vector3(targetUTM.x - leaderUTM.x, targetUTM.y - leaderUTM.y, 0f).Length();
                    float rMin = Math.Max(RadialMinScale * offXY, offXY - RadialMinMargin);

                    var selfPos = currentPositions[mav];
                    float rSelf = new Vector3(selfPos.x - leaderUTM.x, selfPos.y - leaderUTM.y, 0f).Length();

                    if (rSelf < rMin || rDesFromLeader < rMin)
                    {
                        var radial = new Vector3(selfPos.x - leaderUTM.x, selfPos.y - leaderUTM.y, 0f);
                        var rn = radial.Normalized();
                        float push = Math.Max(0f, rMin - rSelf);
                        var pushVec = rn * push;
                        targetUTM = new Vector3(targetUTM.x + pushVec.x, targetUTM.y + pushVec.y, targetUTM.z);
                    }

                    // --- Repulsive obstacle (other followers) ---
                    float dSafe = dSafeBase;
                    Vector3 repulsiveForce = Vector3.Zero;

                    foreach (var otherMav in currentPositions.Keys)
                    {
                        if (otherMav == mav) continue;

                        Vector3 otherPos = currentPositions[otherMav];
                        Vector3 diff = new Vector3(targetUTM.x - otherPos.x, targetUTM.y - otherPos.y, targetUTM.z - otherPos.z);
                        float dist = diff.Length();

                        if (dist < dSafe && dist > 0.01f)
                        {
                            Vector3 dir = diff.Normalized();
                            float strength = RepulsiveBaseGain / (dist * dist); // 1/dist^2
                            repulsiveForce = new Vector3(repulsiveForce.x + dir.x * strength,
                                                         repulsiveForce.y + dir.y * strength,
                                                         repulsiveForce.z + dir.z * strength);
                        }
                    }

                    if (repulsiveForce.Length() > 0f)
                    {
                        targetUTM = new Vector3(targetUTM.x + repulsiveForce.x * RepulsiveScale,
                                                targetUTM.y + repulsiveForce.y * RepulsiveScale,
                                                targetUTM.z + repulsiveForce.z * RepulsiveScale);
                    }

                    // --- Back to WGS84 ---
                    double[] inv = trans.MathTransform.Inverse().Transform(new[] { targetUTM.x, targetUTM.y });
                    var targetGeo = new PointLatLngAlt(inv[1], inv[0], targetUTM.z, "");

                    // --- Send commands ---
                    if (mav.cs.firmware == Firmwares.ArduPlane)
                    {
                        // ===== ORIGINAL ArduPlane PID logic  =====
                        try
                        {
                            // distance and bearing to target
                            var dist = targetGeo.GetDistance(mav.cs.Location);
                            var targyaw = mav.cs.Location.GetBearing(targetGeo);

                            // compute trailer/leader aimpoints as in original logic
                            var targettrailer = targetGeo.newpos(Leader.cs.yaw, Math.Abs(dist) * -0.25);
                            var targetleader = targetGeo.newpos(Leader.cs.yaw, 10 + dist);

                            var yawerror = wrap_180(targyaw - mav.cs.yaw);

                            if (dist < 100)
                            {
                                // when near, aim slightly ahead
                                targyaw = mav.cs.Location.GetBearing(targetleader);
                                yawerror = wrap_180(targyaw - mav.cs.yaw);

                                var targBearing = mav.cs.Location.GetBearing(targetGeo);
                                // if target bearing vs aim bearing disagree >45deg, slow down
                                if (Math.Abs(wrap_180(targBearing - targyaw)) > 45)
                                    dist *= -1;
                            }
                            else
                            {
                                // when far, approach from behind
                                targyaw = mav.cs.Location.GetBearing(targettrailer);
                                yawerror = wrap_180(targyaw - mav.cs.yaw);
                            }

                            // display update (for GCS)
                            mav.GuidedMode.x = (int)(targetGeo.Lat * 1e7);
                            mav.GuidedMode.y = (int)(targetGeo.Lng * 1e7);
                            mav.GuidedMode.z = (float)targetGeo.Alt;

                            // Prepare attitude target message
                            var att_target = new MAVLink.mavlink_set_attitude_target_t
                            {
                                target_system = mav.sysid,
                                target_component = mav.compid,
                                type_mask = 0xff // will clear bits below
                            };

                            // fetch or create PIDs
                            if (!pids.ContainsKey(mav))
                            {
                                pids[mav] = new Tuple<PID, PID, PID, PID>(
                                    new PID(1f, .03f, 0.02f, 10, 20, 0.1f, 0), // roll
                                    new PID(1f, .03f, 0.02f, 10, 20, 0.1f, 0), // pitch
                                    new PID(1, 0, 0.00f, 15, 20, 0.1f, 0),     // yaw
                                    new PID(0.01f, 0.001f, 0, 0.5f, 20, 0.1f, 0) // thrust
                                );
                            }

                            var rollp = pids[mav].Item1;
                            var pitchp = pids[mav].Item2;
                            var yawp = pids[mav].Item3;
                            var thrustp = pids[mav].Item4;

                            double newroll = 0d;
                            double newpitch = 0d;

                            // Pitch from altitude error
                            {
                                var altdelta = targetGeo.Alt - mav.cs.alt;
                                newpitch = altdelta;
                                att_target.type_mask -= 0b00000010; // enable pitch
                                pitchp.set_input_filter_all((float)altdelta);
                                newpitch = pitchp.get_pid();
                            }

                            // Roll from geometry and bearing delta (original logic)
                            {
                                var x = ((Vector3)off).x; // follower's x-offset
                                var leaderturnrad = CurrentState.fromDistDisplayUnit(Leader.cs.radius);
                                var mavturnradius = leaderturnrad - x;

                                var distToTarget = mav.cs.Location.GetDistance(targetGeo);
                                var bearingToTarget = mav.cs.Location.GetBearing(targetGeo);

                                // bearing stability
                                if (distToTarget < 30)
                                    bearingToTarget = mav.cs.Location.GetBearing(targetleader);
                                // fly in from behind
                                if (distToTarget > 100)
                                    bearingToTarget = mav.cs.Location.GetBearing(targettrailer);

                                var bearingDelta = wrap_180(bearingToTarget - mav.cs.yaw);
                                var tangent90 = bearingDelta > 0 ? 90 : -90;

                                newroll = 0;

                                // if the delta is < ~85deg, compute a coordinated turn angle
                                if (Math.Abs(bearingDelta) < 85)
                                {
                                    var insideAngle = Math.Abs(tangent90 - bearingDelta);
                                    var angleCenter = 180 - insideAngle * 2;

                                    // sine rule to estimate turn radius
                                    var sine1 = Math.Max(distToTarget, 40) /
                                                Math.Sin(angleCenter * MathHelper.deg2rad);
                                    var radius = sine1 * Math.Sin(insideAngle * MathHelper.deg2rad);

                                    // average estimated radius with leader radius offset (FF)
                                    radius = (Math.Abs(radius) + Math.Abs(mavturnradius)) / 2;

                                    var angleBank = ((mav.cs.groundspeed * mav.cs.groundspeed) / Math.Max(1.0, radius)) / 9.8;
                                    angleBank *= MathHelper.rad2deg;

                                    newroll = bearingDelta > 0 ? Math.Abs(angleBank) : -Math.Abs(angleBank);
                                }

                                // small proportion on bearing error
                                newroll += MathHelper.constrain(bearingDelta, -20, 20);
                            }

                            // Thrust from distance (with anti-windup)
                            {
                                att_target.type_mask -= 0b01000000; // enable thrust
                                thrustp.set_input_filter_all((float)dist);
                                if (dist > 40) thrustp.reset_I();
                                att_target.thrust = (float)MathHelper.constrain(thrustp.get_pid(), 0.1, 1);
                            }

                            // Compose quaternion with yaw error
                            Quaternion q = Quaternion.from_euler312(newroll * MathHelper.deg2rad, newpitch * MathHelper.deg2rad, yawerror * MathHelper.deg2rad);
                            att_target.q = new float[4];
                            att_target.q[0] = (float)q.q1;
                            att_target.q[1] = (float)q.q2;
                            att_target.q[2] = (float)q.q3;
                            att_target.q[3] = (float)q.q4;

                            // clear type_mask bits for roll/pitch/yaw (use quaternion) and thrust
                            att_target.type_mask -= 0b10000101; // enable attitude (quaternion)

                            port.sendPacket(att_target, mav.sysid, mav.compid);
                        }
                        catch (Exception ex)
                        {
                            Console.WriteLine("Failed to send ArduPlane command " + mav + "\n" + ex);
                        }
                    }
                    else
                    {
                        // ===== COPTER & OTHERS: NEW logic (position + velocity FF, yaw handling when hover) =====
                        if (leaderHover)
                        {
                            // when hover: force yaw to leader's yaw
                            var vzero = new Vector3(0f, 0f, 0f);
                            port.setPositionTargetGlobalInt(
                                mav.sysid, mav.compid,
                                true, true, false, true, // useYaw = true
                                MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT_INT,
                                targetGeo.Lat, targetGeo.Lng, targetGeo.Alt,
                                vzero.x, vzero.y, vzero.z,
                                leaderYawRad, // yaw (radians)
                                0f            // yaw rate
                            );
                        }
                        else
                        {
                            // normal: track position with leader's velocity feedforward (no yaw control)
                            var vff = new Vector3((float)Leader.cs.vx, (float)Leader.cs.vy, (float)Leader.cs.vz);
                            port.setPositionTargetGlobalInt(
                                mav.sysid, mav.compid,
                                true, true, false, false, // useYaw = false
                                MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT_INT,
                                targetGeo.Lat, targetGeo.Lng, targetGeo.Alt,
                                vff.x, vff.y, vff.z,
                                0f, 0f
                            );
                        }
                    }
                }
            }
        }

        public bool gimbal { get; set; }
    }

    /// <summary>PID controller (Simplified)</summary>
    internal class PID
    {
        private float dt, input, deriv, integrator, imax, filtHz;
        private readonly float kp, ki, kd;
        private bool resetFilter = true;
        public PID(float p, float i, float d, float im, float fHz, float dt0, float ff)
        {
            kp = p; ki = i; kd = d; imax = Math.Abs(im);
            filtHz = Math.Max(fHz, 0.01f); dt = dt0;
        }
        public void set_input_filter_all(float inVal)
        {
            if (float.IsInfinity(inVal)) return;
            if (resetFilter) { input = inVal; deriv = 0; resetFilter = false; }
            float alpha = dt / (dt + 1f / (2f * (float)Math.PI * filtHz));
            float delta = alpha * (inVal - input);
            input += delta;
            deriv = dt > 0 ? delta / dt : 0;
        }
        public float get_pid()
        {
            float P = kp * input;
            integrator = Math.Max(-imax, Math.Min(imax, integrator + ki * input * dt));
            float D = kd * deriv;
            return P + integrator + D;
        }
        public void reset_I() { integrator = 0; }
    }
}
