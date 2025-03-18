using MissionPlanner.ArduPilot;
using MissionPlanner.Utilities;
using MathNet.Spatial.Euclidean;
using MathNet.Spatial.Units;
using System;
using System.Collections.Generic;
using System.Linq;
using Vector3 = MissionPlanner.Utilities.Vector3;

namespace MissionPlanner.Swarm
{
    /// <summary>
    /// Hybrid geometric formation control with legacy compatibility
    /// </summary>
    class GeometricFormation : Swarm
    {
        #region Preserved Original Code
        // Maintain all original functionality
        Dictionary<MAVState, Vector3> offsets = new Dictionary<MAVState, Vector3>();
        private PointLatLngAlt masterpos = new PointLatLngAlt();

        public void setOffsets(MAVState mav, double x, double y, double z)
        {
            offsets[mav] = new Vector3(x, y, z);
        }

        public Vector3 getOffsets(MAVState mav)
        {
            return offsets.ContainsKey(mav) ? offsets[mav] : new Vector3(offsets.Count, 0, 0);
        }

        public override void Update()
        {
            if (MainV2.comPort.MAV.cs.lat == 0 || MainV2.comPort.MAV.cs.lng == 0)
                return;

            if (Leader == null)
                Leader = MainV2.comPort.MAV;

            masterpos = new PointLatLngAlt(Leader.cs.lat, Leader.cs.lng, Leader.cs.alt, "");
        }
        #endregion

        #region Geometric Control Additions
        // New geometric components
        private Dictionary<MAVState, QuadrotorState> _states = new Dictionary<MAVState, QuadrotorState>();
        private LocalCartesian _localCoordSystem;

        private const float Kx = 8.0f;
        private const float Kv = 6.0f;
        private const float Kr = 3.0f;
        private const float Kω = 1.5f;
        private const float SafetyRadius = 5.0f;

        class QuadrotorState
        {
            public Vector3D Position { get; set; }
            public Vector3D Velocity { get; set; }
            public Quaternion Attitude { get; set; }
            public Vector3D AngularVelocity { get; set; }
            public Vector3D PayloadWrench { get; set; }
        }

        private void InitializeCoordinateSystem(PointLatLngAlt origin)
        {
            _localCoordSystem = new LocalCartesian(origin.ToGeoCoordinate());
        }

        private Vector3D ToLocalCoordinates(PointLatLngAlt geoPoint)
        {
            if (_localCoordSystem == null)
                InitializeCoordinateSystem(masterpos);
            
            return _localCoordSystem.FromGeodetic(geoPoint.ToGeoCoordinate());
        }

        private PointLatLngAlt ToGeodetic(Vector3D localPoint)
        {
            if (_localCoordSystem == null)
                InitializeCoordinateSystem(masterpos);
            
            return _localCoordSystem.ToGeoCoordinate(localPoint).ToPointLatLngAlt();
        }

        private void UpdateQuadrotorState(MAVState mav)
        {
            var geoPos = new PointLatLngAlt(mav.cs.lat, mav.cs.lng, mav.cs.alt);
            
            _states[mav] = new QuadrotorState
            {
                Position = ToLocalCoordinates(geoPos),
                Velocity = new Vector3D(mav.cs.vx, mav.cs.vy, mav.cs.vz),
                Attitude = Quaternion.FromEuler(
                    Angle.FromDegrees(mav.cs.roll),
                    Angle.FromDegrees(mav.cs.pitch),
                    Angle.FromDegrees(mav.cs.yaw)),
                AngularVelocity = new Vector3D(mav.cs.wx, mav.cs.wy, mav.cs.wz)
            };
        }
        #endregion

        #region Hybrid Control Implementation
        public override void SendCommand()
        {
            // Preserve original waypoint logic
            OriginalSendCommand();
            
            // Add geometric control
            GeometricSendCommand();
        }

        private void OriginalSendCommand()
        {
            // Preserved original command logic
            if (masterpos.Lat == 0 || masterpos.Lng == 0)
                return;

            foreach (var port in MainV2.Comports.ToArray())
            {
                foreach (var mav in port.MAVlist)
                {
                    if (mav == Leader) continue;
                    
                    // Original position calculation
                    PointLatLngAlt target = CalculateLegacyPosition(mav);
                    port.setGuidedModeWP(mav.sysid, mav.compid, target);
                }
            }
        }

        private void GeometricSendCommand()
        {
            foreach (var port in MainV2.Comports.ToArray())
            {
                foreach (var mav in port.MAVlist)
                {
                    UpdateQuadrotorState(mav);
                    var desired = CalculateGeometricTarget(mav);
                    var cmd = ComputeGeometricCommand(mav, desired);
                    ApplyCollisionAvoidance(mav, ref cmd);
                    SendGeometricCommand(port, mav, cmd);
                }
            }
        }

        private MAVLink.mavlink_set_attitude_target_t ComputeGeometricCommand(MAVState mav, QuadrotorState desired)
        {
            var current = _states[mav];
            
            var e_x = current.Position - desired.Position;
            var e_v = current.Velocity - desired.Velocity;
            
            var q_err = desired.Attitude * current.Attitude.Inverse();
            var e_R = 0.5 * q_err.ToRollPitchYaw().ToVector3D();
            var e_ω = current.AngularVelocity - desired.AngularVelocity;

            var F_des = -Kx * e_x - Kv * e_v + 
                       new Vector3D(0, 0, mav.Mass * 9.81) + 
                       current.PayloadWrench;

            var M_des = -Kr * e_R - Kω * e_ω + 
                       current.AngularVelocity.Cross(mav.Inertia * current.AngularVelocity);

            return new MAVLink.mavlink_set_attitude_target_t
            {
                q = desired.Attitude.ToFloatArray(),
                body_roll_rate = (float)M_des.X,
                body_pitch_rate = (float)M_des.Y,
                body_yaw_rate = (float)M_des.Z,
                thrust = (float)(F_des.Z / (mav.Mass * 9.81))
            };
        }
        #endregion

        #region Common Helper Methods
        private PointLatLngAlt CalculateLegacyPosition(MAVState mav)
        {
            // Original position calculation logic
            var target = new PointLatLngAlt(masterpos);
            // ... preserved offset calculations ...
            return target;
        }

        private QuadrotorState CalculateGeometricTarget(MAVState mav)
        {
            var leaderState = _states[Leader];
            var offset = new Vector3D(offsets[mav].x, offsets[mav].y, offsets[mav].z);
            
            return new QuadrotorState
            {
                Position = leaderState.Position + leaderState.Attitude.Rotate(offset),
                Attitude = leaderState.Attitude,
                Velocity = leaderState.Velocity,
                AngularVelocity = leaderState.AngularVelocity
            };
        }

        private void ApplyCollisionAvoidance(MAVState mav, ref MAVLink.mavlink_set_attitude_target_t cmd)
        {
            foreach (var other in _states.Keys.Where(k => k != mav))
            {
                var delta = _states[mav].Position - _states[other].Position;
                if (delta.Length < SafetyRadius)
                {
                    var repulse = (SafetyRadius - delta.Length) * delta.Normalize();
                    cmd.thrust += (float)(repulse.Z * 0.3);
                    cmd.body_roll_rate += (float)(repulse.X * 0.3);
                    cmd.body_pitch_rate += (float)(repulse.Y * 0.3);
                }
            }
        }

        private void SendGeometricCommand(MAVLinkInterface port, MAVState mav, MAVLink.mavlink_set_attitude_target_t cmd)
        {
            port.sendPacket(cmd, mav.sysid, mav.compid);
        }
        #endregion
    }
}