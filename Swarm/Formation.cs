using System;
using System.Collections.Generic;

namespace MissionPlanner.Utilities
{
    // Minimal Vector2 class.
    public class Vector2
    {
        public float x, y;
        public Vector2(float x, float y)
        {
            this.x = x;
            this.y = y;
        }
        public static Vector2 operator +(Vector2 a, Vector2 b) 
            => new Vector2(a.x + b.x, a.y + b.y);
        public static Vector2 operator -(Vector2 a, Vector2 b) 
            => new Vector2(a.x - b.x, a.y - b.y);
        public static Vector2 operator /(Vector2 a, float d) 
            => new Vector2(a.x / d, a.y / d);
    }

    // Minimal Vector3 class.
    public class Vector3
    {
        public float x, y, z;
        public Vector3(float x, float y, float z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }
        public static Vector3 Zero => new Vector3(0f, 0f, 0f);
        public static Vector3 operator +(Vector3 a, Vector3 b) 
            => new Vector3(a.x + b.x, a.y + b.y, a.z + b.z);
        public static Vector3 operator -(Vector3 a, Vector3 b) 
            => new Vector3(a.x - b.x, a.y - b.y, a.z - b.z);
        public static Vector3 operator *(float d, Vector3 a) 
            => new Vector3(a.x * d, a.y * d, a.z * d);
        public static Vector3 operator *(Vector3 a, float d) 
            => new Vector3(a.x * d, a.y * d, a.z * d);
        public static float Dot(Vector3 a, Vector3 b) 
            => a.x * b.x + a.y * b.y + a.z * b.z;

        // Cross product.
        public static Vector3 Cross(Vector3 a, Vector3 b)
        {
            return new Vector3(
                a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x
            );
        }
    }

    // Minimal PointLatLngAlt class.
    public class PointLatLngAlt
    {
        public double Lat { get; set; }
        public double Lng { get; set; }
        public double Alt { get; set; }
        public string Name { get; set; }
        public PointLatLngAlt(double lat, double lng, double alt, string name)
        {
            Lat = lat;
            Lng = lng;
            Alt = alt;
            Name = name;
        }
    }

    // Minimal MathHelper class.
    public static class MathHelper
    {
        public const float deg2rad = (float)(Math.PI / 180.0);
        public static float constrain(float value, float min, float max)
        {
            if (value < min) return min;
            if (value > max) return max;
            return value;
        }
    }

    // Minimal Quaternion class.
    public class Quaternion
    {
        public double q1, q2, q3, q4;
        public Quaternion(double q1, double q2, double q3, double q4)
        {
            this.q1 = q1;
            this.q2 = q2;
            this.q3 = q3;
            this.q4 = q4;
        }
        // Create a quaternion from Euler angles using a 3-1-2 sequence.
        public static Quaternion from_euler312(double roll, double pitch, double yaw)
        {
            double cr = Math.Cos(roll * 0.5);
            double sr = Math.Sin(roll * 0.5);
            double cp = Math.Cos(pitch * 0.5);
            double sp = Math.Sin(pitch * 0.5);
            double cy = Math.Cos(yaw * 0.5);
            double sy = Math.Sin(yaw * 0.5);
            double q1 = cr * cp * cy + sr * sp * sy;
            double q2 = sr * cp * cy - cr * sp * sy;
            double q3 = cr * sp * cy + sr * cp * sy;
            double q4 = cr * cp * sy - sr * sp * cy;
            return new Quaternion(q1, q2, q3, q4);
        }
    }
}

namespace MissionPlanner.ArduPilot
{
    using MissionPlanner.Utilities;
    // Minimal MAVState stub.
    public class MAVState
    {
        public ControlState cs = new ControlState();
        public int sysid;
        public int compid;
    }
    
    public class ControlState
    {
        public double lat, lng, alt;
        public double yaw;
        public float vx, vy, vz;
        public Vector3 Location => new Vector3((float)lat, (float)lng, (float)alt);
    }
}

namespace MissionPlanner.Swarm
{
    using MissionPlanner.Utilities;
    using MissionPlanner.ArduPilot;

    // Abstract base class for the formation controller.
    public abstract class SwarmBase
    {
        // Renamed ambiguous property to FormationLeader.
        public MAVState FormationLeader { get; set; }
        public abstract void Update();
        public abstract void SendCommand();
    }

    // Minimal PID stub.
    public class PID { }

    // Minimal LoadAttitudeController stub.
    public class LoadAttitudeController
    {
        public virtual Vector3 CompensateRigidBodyDynamics(MAVState leader, MAVState follower, Dictionary<MAVState, Vector3> offsets)
        {
            // Base implementation returns zero.
            return Vector3.Zero;
        }
    }

    // PayloadState holds the estimated payload state.
    public class PayloadState
    {
        public PointLatLngAlt Position { get; set; }
        public float Yaw { get; set; }
        public Vector3 Velocity { get; set; }
        public Vector3 Acceleration { get; set; }
        public Matrix3x3 RotationMatrix { get; set; }
        public Vector3 AngularVelocity { get; set; }
        public Vector3 AngularAcceleration { get; set; }
    }

    // Simple 3x3 matrix class.
    public class Matrix3x3
    {
        public float[,] Data { get; private set; }
        public Matrix3x3(float[,] data)
        {
            Data = data;
        }
        public static Matrix3x3 Identity()
        {
            return new Matrix3x3(new float[,] { { 1f, 0f, 0f }, { 0f, 1f, 0f }, { 0f, 0f, 1f } });
        }
        public Vector3 Multiply(Vector3 v)
        {
            float x = Data[0, 0] * v.x + Data[0, 1] * v.y + Data[0, 2] * v.z;
            float y = Data[1, 0] * v.x + Data[1, 1] * v.y + Data[1, 2] * v.z;
            float z = Data[2, 0] * v.x + Data[2, 1] * v.y + Data[2, 2] * v.z;
            return new Vector3(x, y, z);
        }
        public static Matrix3x3 Transpose(Matrix3x3 m)
        {
            float[,] t = new float[3, 3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    t[i, j] = m.Data[j, i];
            return new Matrix3x3(t);
        }
    }

    // PayloadEstimator estimates payload state based on three MAVStates.
    public class PayloadEstimator
    {
        private const float sideLength = 0.5f;   // 50 cm
        private readonly float R;                // Circumradius: sideLength/sqrt(3)

        public PayloadEstimator()
        {
            R = sideLength / (float)Math.Sqrt(3.0);
        }

        public PayloadState EstimatePayloadState(List<MAVState> drones)
        {
            if (drones.Count < 3)
                throw new ArgumentException("At least 3 drones are required for payload estimation.");

            Vector3[] dronePos = new Vector3[3];
            Vector2[] droneHoriz = new Vector2[3];
            for (int i = 0; i < 3; i++)
            {
                dronePos[i] = drones[i].cs.Location;
                droneHoriz[i] = new Vector2(dronePos[i].x, dronePos[i].y);
            }

            // Define payload attachment points in the payload body frame.
            Vector2[] payloadVertices = new Vector2[3];
            payloadVertices[0] = new Vector2(0f, R);
            payloadVertices[1] = new Vector2(sideLength / 2f, -R);
            payloadVertices[2] = new Vector2(-sideLength / 2f, -R);

            // Compute centroids.
            Vector2 centroidPayload = (payloadVertices[0] + payloadVertices[1] + payloadVertices[2]) / 3f;
            Vector2 centroidDrones = (droneHoriz[0] + droneHoriz[1] + droneHoriz[2]) / 3f;

            // Estimate yaw using least-squares alignment.
            float num = 0f, den = 0f;
            for (int i = 0; i < 3; i++)
            {
                Vector2 dp = new Vector2(payloadVertices[i].x - centroidPayload.x, payloadVertices[i].y - centroidPayload.y);
                Vector2 dd = new Vector2(droneHoriz[i].x - centroidDrones.x, droneHoriz[i].y - centroidDrones.y);
                num += dp.x * dd.y - dp.y * dd.x;
                den += dp.x * dd.x + dp.y * dd.y;
            }
            float theta = (float)Math.Atan2(num, den);

            // Estimate payload altitude.
            float avgAlt = (dronePos[0].z + dronePos[1].z + dronePos[2].z) / 3f;
            float payloadAlt = avgAlt + 2f; // add cable length (2 m)
            PointLatLngAlt pos = new PointLatLngAlt(centroidDrones.x, centroidDrones.y, payloadAlt, "");

            // Build rotation matrix from yaw (assume zero pitch/roll).
            float cosT = (float)Math.Cos(theta);
            float sinT = (float)Math.Sin(theta);
            float[,] rData = new float[3, 3] { { cosT, -sinT, 0f }, { sinT, cosT, 0f }, { 0f, 0f, 1f } };
            Matrix3x3 R_L = new Matrix3x3(rData);

            PayloadState state = new PayloadState();
            state.Position = pos;
            state.Yaw = theta;
            state.Velocity = Vector3.Zero;         // In a real system, these would be estimated over time.
            state.Acceleration = Vector3.Zero;
            state.RotationMatrix = R_L;
            state.AngularVelocity = Vector3.Zero;
            state.AngularAcceleration = Vector3.Zero;
            return state;
        }
    }

    // TensionSolver computes individual cable force contributions.
    public class TensionSolver
    {
        private List<Vector3> rVectors;
        public TensionSolver(List<Vector3> rVectors)
        {
            this.rVectors = rVectors;
        }

        // Simplified equal load-sharing solution.
        // NOTE: In a full implementation, you would solve a system that includes both F_d and tau_d.
        public List<Vector3> Solve(Vector3 F_d, Vector3 tau_d)
        {
            int n = rVectors.Count;
            List<Vector3> forces = new List<Vector3>();
            for (int i = 0; i < n; i++)
            {
                forces.Add((-1.0f / n) * F_d); // Only distributing translational force equally.
            }
            return forces;
        }
    }

    // EnhancedLoadAttitudeController integrates full payload dynamics (translational and simplified rotational) and hybrid tension compensation.
    public class EnhancedLoadAttitudeController : LoadAttitudeController
    {
        public float payloadMass = 0.5f; // Nominal payload mass (kg)
        public Matrix3x3 payloadInertia = new Matrix3x3(new float[,] { { 0.05f, 0f, 0f }, { 0f, 0.05f, 0f }, { 0f, 0f, 0.05f } });
        public PayloadState EstimatedPayloadState { get; set; }
        public TensionSolver tensionSolver;
        public float TensionThreshold = 0.5f; // Newtons

        public EnhancedLoadAttitudeController()
        {
            // Define attachment points in the payload body frame.
            List<Vector3> rVectors = new List<Vector3>();
            float R_val = 0.5f / (float)Math.Sqrt(3.0);
            rVectors.Add(new Vector3(0f, R_val, 0f));
            rVectors.Add(new Vector3((0.5f / 2f), -R_val, 0f));
            rVectors.Add(new Vector3(-(0.5f / 2f), -R_val, 0f));
            tensionSolver = new TensionSolver(rVectors);
        }

        // Compute desired net force (F_d) and desired net torque (tau_d) for the payload.
        public void ComputeDesiredTerms(out Vector3 F_d, out Vector3 tau_d)
        {
            // Translational term: F_d = m*(a_payload + g)
            Vector3 a = EstimatedPayloadState.Acceleration; // Estimated payload acceleration (zero if unknown)
            Vector3 g = new Vector3(0f, 0f, 9.81f);
            F_d = payloadMass * (a + g);

            // Rotational term: tau_d = J*(angularAcceleration) + ω x (J*ω)
            Vector3 angularAcc = EstimatedPayloadState.AngularAcceleration; // Estimated angular acceleration (zero if unknown)
            Vector3 omega = EstimatedPayloadState.AngularVelocity;            // Estimated angular velocity
            Vector3 J_angularAcc = payloadInertia.Multiply(angularAcc);
            Vector3 J_omega = payloadInertia.Multiply(omega);
            Vector3 omegaCrossJomega = Vector3.Cross(omega, J_omega);
            tau_d = J_angularAcc + omegaCrossJomega;
        }

        public override Vector3 CompensateRigidBodyDynamics(MAVState leader, MAVState follower, Dictionary<MAVState, Vector3> offsets)
        {
            Vector3 baseComp = base.CompensateRigidBodyDynamics(leader, follower, offsets);
            Vector3 F_d, tau_d;
            ComputeDesiredTerms(out F_d, out tau_d);
            List<Vector3> cableForces = tensionSolver.Solve(F_d, tau_d);

            // Determine follower index.
            int index = 0;
            foreach (var kv in offsets)
            {
                if (kv.Key == follower)
                    break;
                index++;
            }
            Vector3 feedforwardTerm = cableForces[index];

            // Hybrid tension compensation: if the projected tension is below a threshold, assume cable slack.
            float estimatedTension = EstimateCableTension(feedforwardTerm, follower);
            if (estimatedTension < TensionThreshold)
            {
                feedforwardTerm = Vector3.Zero;
            }
            Vector3 totalComp = baseComp + feedforwardTerm;
            return totalComp;
        }

        public float EstimateCableTension(Vector3 feedforwardTerm, MAVState mav)
        {
            Vector3 q = GetCableUnitVector(mav);
            return Math.Abs(Vector3.Dot(feedforwardTerm, q));
        }

        // For this example, assume the cable unit vector is vertically downward.
        public Vector3 GetCableUnitVector(MAVState mav)
        {
            return new Vector3(0f, 0f, -1f);
        }
    }

    // Formation controller integrates payload estimation and sends commands.
    public class Formation : SwarmBase
    {
        private Dictionary<MAVState, Vector3> offsets = new Dictionary<MAVState, Vector3>();
        private Dictionary<MAVState, Vector3> compFiltered = new Dictionary<MAVState, Vector3>();
        private PointLatLngAlt masterpos = new PointLatLngAlt(0, 0, 0, "");
        private EnhancedLoadAttitudeController payloadController = new EnhancedLoadAttitudeController();
        private PayloadEstimator payloadEstimator = new PayloadEstimator();

        private float payloadGain = 0.5f;
        private float maxRollComp = 10f;
        private float maxPitchComp = 10f;
        private float compTau = 0.2f;
        private float blendAlpha = 0.7f;

        // Set offsets for each MAV.
        public void setOffsets(MAVState mav, double x, double y, double z)
        {
            offsets[mav] = new Vector3((float)x, (float)y, (float)z);
        }

        public Vector3 getOffsets(MAVState mav)
        {
            return offsets.ContainsKey(mav) ? offsets[mav] : new Vector3(offsets.Count, 0f, 0f);
        }

        public override void Update()
        {
            if (FormationLeader == null)
                FormationLeader = new MAVState(); // For demonstration.
            masterpos = new PointLatLngAlt(FormationLeader.cs.lat, FormationLeader.cs.lng, FormationLeader.cs.alt, "");

            // Use the first 3 drones from offsets for payload estimation.
            List<MAVState> droneList = new List<MAVState>();
            foreach (var kv in offsets)
            {
                droneList.Add(kv.Key);
                if (droneList.Count == 3)
                    break;
            }
            if (droneList.Count == 3)
            {
                PayloadState estState = payloadEstimator.EstimatePayloadState(droneList);
                payloadController.EstimatedPayloadState = estState;
            }
        }

        public override void SendCommand()
        {
            if (masterpos == null)
                return;

            foreach (var kv in offsets)
            {
                MAVState mav = kv.Key;
                if (mav == FormationLeader)
                    continue;

                PointLatLngAlt target = masterpos;
                try
                {
                    float heading = -(float)(FormationLeader.cs.yaw * MathHelper.deg2rad);
                    Vector3 offset = getOffsets(mav);

                    // Compute payload compensation.
                    Vector3 payloadComp = payloadController.CompensateRigidBodyDynamics(FormationLeader, mav, offsets);

                    // Filter the compensation command (simple first-order low-pass filter).
                    float dt = (float)(DateTime.UtcNow - DateTime.MinValue).TotalSeconds;
                    Vector3 prev = compFiltered.ContainsKey(mav) ? compFiltered[mav] : Vector3.Zero;
                    float alpha = dt / (compTau + dt);
                    Vector3 filtered = new Vector3(
                        prev.x + (payloadComp.x - prev.x) * alpha,
                        prev.y + (payloadComp.y - prev.y) * alpha,
                        prev.z + (payloadComp.z - prev.z) * alpha
                    );
                    compFiltered[mav] = filtered;
                    float rollComp = MathHelper.constrain(filtered.x * payloadGain, -maxRollComp, maxRollComp);
                    float pitchComp = MathHelper.constrain(filtered.y * payloadGain, -maxPitchComp, maxPitchComp);
                    float newroll = rollComp * blendAlpha;
                    float newpitch = pitchComp * blendAlpha;
                    double targyaw = 0; // Compute target yaw as needed.
                    double yawerror = wrap_180(targyaw - FormationLeader.cs.yaw);
                    Quaternion q = Quaternion.from_euler312(newroll * MathHelper.deg2rad, newpitch * MathHelper.deg2rad, (float)yawerror * MathHelper.deg2rad);

                    Console.WriteLine("Sending command to MAV {0}: Roll {1}, Pitch {2}, Yaw Error {3}",
                        mav.sysid, newroll, newpitch, yawerror);
                }
                catch (Exception ex)
                {
                    Console.WriteLine("SendCommand failed: " + ex);
                }
            }
        }

        private double wrap_180(double input)
        {
            if (input > 180)
                return input - 360;
            if (input < -180)
                return input + 360;
            return input;
        }
    }
}
