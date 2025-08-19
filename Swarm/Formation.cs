using System;
using System.Collections.Generic;
using MissionPlanner.ArduPilot;
using MissionPlanner.Utilities;
using ProjNet.CoordinateSystems;
using ProjNet.CoordinateSystems.Transformations;
using GeoAPI.CoordinateSystems;
using GeoAPI.CoordinateSystems.Transformations;
using MathNet.Numerics.LinearAlgebra;
using Vector3 = MissionPlanner.Utilities.Vector3;

namespace MissionPlanner.Swarm
{
    // --- Extension methods สำหรับ Vector3 ---
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

    internal class Formation : Swarm
    {
        private readonly Dictionary<MAVState, Vector3> offsets = new Dictionary<MAVState, Vector3>();
        private readonly Dictionary<MAVState, Vector3> compFiltered = new Dictionary<MAVState, Vector3>();
        private readonly Dictionary<MAVState, float> prevYawRate = new Dictionary<MAVState, float>();
        private readonly LoadAttitudeController payloadController = new LoadAttitudeController();

        // --- เกนพื้นฐาน (ปรับได้) ---
        private float payloadGain = 0.5f;
        private float Kp = 1.0f;
        private float Kv = 0.5f;

        private const float gammaPayload = 0.1f;
        private const float gammaKp = 0.05f;
        private const float gammaKv = 0.05f;
        private const float compTau = 0.025f;

        private readonly CoordinateTransformationFactory ctfac = new CoordinateTransformationFactory();
        private readonly IGeographicCoordinateSystem wgs84 = GeographicCoordinateSystem.WGS84;

        private PointLatLngAlt masterpos;
        private DateTime lastUpdateTime = DateTime.UtcNow;

        // ===== ค่าคงที่สำหรับการเลี้ยว/กันชน/กันตัดเข้า =====
        private const float DEG2RAD = (float)(Math.PI / 180.0);

        // นำเป้าหมายล่วงหน้า (ช่วยไม่ตกท้ายตอนเลี้ยว)
        private const float TurnLeadTime = 0.5f;        // s

        // Blend เข้าสู่โหมดเกาะวง
        private const float TurnBlendStartRate = 8f;    // deg/s
        private const float TurnBlendMaxRate   = 45f;   // deg/s

        // ขยายรัศมีของเป้าหมายตามมุมเลี้ยวและขนาด offset (ช่วยลำวงนอก)
        private const float OuterDilatePerDeg = 0.011f; // m per |deg/s| * |offset|

        // กันตัดเข้าวงในเมื่อหยุดเลี้ยว / ออกจากโค้ง
        private const float RadialMinMargin = 1.0f;     // m
        private const float RadialMinScale  = 0.80f;    // rMin >= 0.8*|offsetXY|

        // กันชนพื้นฐานแบบผลัก 1/dist^2
        private const float dSafeBase         = 2.0f;   // m
        private const float CriticalRadius    = 1.0f;   // m (ใช้ในแบบขยาย gain ได้ถ้าต้องการ)
        private const float RepulsiveBaseGain = 1.3f;   // scale of 1/dist^2
        private const float RepulsiveScale    = 1.0f;   // สเกลรวม

        // ==== Hover detection thresholds (ให้ทุกลำหันหัวตาม leader เมื่อ hover) ====
        private const float HoverSpeedXYThresh    = 0.25f; // m/s
        private const float HoverVzThresh         = 0.25f; // m/s
        private const float HoverYawRateThreshDeg = 3f;    // deg/s

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

        public override void SendCommand()
        {
            if (Leader == null || masterpos.Lat == 0) return;

            // 1) แปลงเป็น UTM
            int zone = (int)((masterpos.Lng + 180.0) / 6.0);
            var utm = ProjectedCoordinateSystem.WGS84_UTM(zone, masterpos.Lat >= 0);
            var trans = ctfac.CreateFromCoordinateSystems(wgs84, utm);
            double[] lXY = trans.MathTransform.Transform(new[] { masterpos.Lng, masterpos.Lat });
            Vector3 leaderUTM = new Vector3((float)lXY[0], (float)lXY[1], (float)masterpos.Alt);

            // 2) ตัวแปรสำหรับ hover / การเลี้ยว
            float leaderVxy = (float)Math.Sqrt(Leader.cs.vx * Leader.cs.vx + Leader.cs.vy * Leader.cs.vy);
            float leaderYawRateDeg = payloadController.EstimateYawRate(Leader);
            bool leaderHover =
                leaderVxy < HoverSpeedXYThresh &&
                Math.Abs(Leader.cs.vz) < HoverVzThresh &&
                Math.Abs(leaderYawRateDeg) < HoverYawRateThreshDeg;

            float leaderYawRad = (float)(Leader.cs.yaw * Math.PI / 180.0);

            // 3) สำหรับ turn-aware (ICC)
            float yawAbs = Math.Abs(leaderYawRateDeg);
            float yawRateRad = Math.Max(1e-3f, yawAbs * DEG2RAD);
            float leadSpeed = Math.Max(0.01f, leaderVxy);
            float Rturn = leadSpeed / yawRateRad;

            float yawRadHeading = (float)(-Leader.cs.yaw * Math.PI / 180.0);
            var fwd = new Vector3((float)Math.Cos(yawRadHeading), (float)Math.Sin(yawRadHeading), 0f);
            var left = new Vector3(-fwd.y, fwd.x, 0f);
            var nrm = (leaderYawRateDeg >= 0) ? left : new Vector3(-left.x, -left.y, 0f);
            var ICC = new Vector3(leaderUTM.x + nrm.x * Rturn, leaderUTM.y + nrm.y * Rturn, leaderUTM.z);

            // 4) เก็บตำแหน่งปัจจุบันของทุกลำ
            Dictionary<MAVState, Vector3> currentPositions = new Dictionary<MAVState, Vector3>();
            foreach (var port0 in MainV2.Comports)
            {
                foreach (var mav0 in port0.MAVlist)
                {
                    double[] cXY = trans.MathTransform.Transform(new[] { mav0.cs.lng, mav0.cs.lat });
                    currentPositions[mav0] = new Vector3((float)cXY[0], (float)cXY[1], mav0.cs.alt);
                }
            }

            // 5) วนสั่งการ
            foreach (var port in MainV2.Comports)
            {
                foreach (var mav in port.MAVlist)
                {
                    if (mav == Leader) continue;

                    // 5.1) Target เริ่มต้น: offset ที่หมุนตาม heading + นำหน้า TurnLeadTime
                    Vector3 off = getOffsets(mav);
                    float yawPreview = yawRadHeading + leaderYawRateDeg * DEG2RAD * TurnLeadTime;

                    Vector3 targetUTM = new Vector3(
                        leaderUTM.x + off.x * (float)Math.Cos(yawPreview) - off.y * (float)Math.Sin(yawPreview),
                        leaderUTM.y + off.x * (float)Math.Sin(yawPreview) + off.y * (float)Math.Cos(yawPreview),
                        (float)masterpos.Alt + off.z
                    );

                    // 5.2) Turn-aware: พาไปเกาะวง + ขยายรัศมีฝั่งนอก (เฉพาะตอนเลี้ยวจริง)
                    if (yawAbs >= TurnBlendStartRate && leadSpeed > 0.5f)
                    {
                        Vector3 vecL = new Vector3(leaderUTM.x - ICC.x, leaderUTM.y - ICC.y, 0f);
                        Vector3 vecOffWorld = new Vector3(targetUTM.x - leaderUTM.x, targetUTM.y - leaderUTM.y, 0f);
                        Vector3 vecF = new Vector3(vecL.x + vecOffWorld.x, vecL.y + vecOffWorld.y, 0f);

                        float rF = Math.Max(0.1f, vecF.Length());

                        float extraR = off.Length() * OuterDilatePerDeg * yawAbs; // ขยายรัศมีช่วยลำวงนอก
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

                    // 5.3) Radial barrier: ป้องกัน “ตัดเข้าวงใน” เมื่อหยุดเลี้ยว/ออกจากโค้ง
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

                    // 5.4) กันชนแบบ repulsive (จากตำแหน่งลำอื่น ๆ) — 1/dist^2
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

                    // 5.5) แปลงกลับ Lat/Lng
                    double[] inv = trans.MathTransform.Inverse().Transform(new[] { targetUTM.x, targetUTM.y });
                    var targetGeo = new PointLatLngAlt(inv[1], inv[0], targetUTM.z, "");

                    // 5.6) ส่งคำสั่ง
                    if (mav.cs.firmware == Firmwares.ArduPlane)
                    {
                        // ===== ฝั่ง Plane: ใช้ FF+PD+ชดเชย และ "หันตาม leader เมื่อ hover" =====
                        DateTime now = DateTime.UtcNow;
                        float dt = (float)(now - lastUpdateTime).TotalSeconds;

                        Vector3 aL = new Vector3(Leader.cs.ax, Leader.cs.ay, Leader.cs.az);
                        float yawR = payloadController.EstimateYawRate(Leader);
                        float yawA = prevYawRate.TryGetValue(mav, out var pr) ? (yawR - pr) / Math.Max(1e-3f, dt) : 0f;
                        prevYawRate[mav] = yawR;

                        Vector3 a_cent = new Vector3(-yawR * yawR * off.x, -yawR * yawR * off.y, 0f);
                        Vector3 a_tan  = new Vector3(-off.y * yawA,       off.x * yawA,       0f);
                        Vector3 a_ff   = new Vector3(aL.x + a_cent.x + a_tan.x, aL.y + a_cent.y + a_tan.y, aL.z + a_cent.z + a_tan.z);

                        double[] cXY = trans.MathTransform.Transform(new[] { mav.cs.lng, mav.cs.lat });
                        Vector3 curUTM = new Vector3((float)cXY[0], (float)cXY[1], (float)mav.cs.alt);

                        Vector3 e_pos = new Vector3(targetUTM.x - curUTM.x, targetUTM.y - curUTM.y, targetUTM.z - curUTM.z);
                        Vector3 curV  = new Vector3((float)mav.cs.vx, (float)mav.cs.vy, (float)mav.cs.vz);
                        Vector3 v_ff  = new Vector3((float)Leader.cs.vx - yawR * off.y, (float)Leader.cs.vy + yawR * off.x, (float)Leader.cs.vz);
                        Vector3 e_vel = new Vector3(v_ff.x - curV.x, v_ff.y - curV.y, v_ff.z - curV.z);
                        Vector3 a_fb  = new Vector3(e_pos.x * Kp + e_vel.x * Kv, e_pos.y * Kp + e_vel.y * Kv, e_pos.z * Kp + e_vel.z * Kv);

                        Vector3 partComp = payloadController.CompensateRigidBodyDynamics(Leader, mav, offsets);
                        float alpha = dt / (compTau + dt);
                        Vector3 prevC = compFiltered.ContainsKey(mav) ? compFiltered[mav] : Vector3.Zero;
                        Vector3 smooth = new Vector3(
                            prevC.x + (partComp.x - prevC.x) * alpha,
                            prevC.y + (partComp.y - prevC.y) * alpha,
                            prevC.z + (partComp.z - prevC.z) * alpha
                        );
                        compFiltered[mav] = smooth;

                        Vector3 u = new Vector3(a_ff.x + a_fb.x + smooth.x * payloadGain,
                                                a_ff.y + a_fb.y + smooth.y * payloadGain,
                                                a_ff.z + a_fb.z + smooth.z * payloadGain);

                        double phi = Math.Asin(Math.Max(-1, Math.Min(1, u.y / 9.81))) * (180.0 / Math.PI);
                        double tht = Math.Asin(Math.Max(-1, Math.Min(1, -u.x / 9.81))) * (180.0 / Math.PI);
                        float thrust = (float)Math.Max(0.1, Math.Min(1, (u.z + 9.81) / 9.81));

                        float tensionError = (float)(e_pos.x * smooth.x + e_pos.y * smooth.y + e_pos.z * smooth.z);
                        float posErrorSq = (float)(e_pos.x * e_pos.x + e_pos.y * e_pos.y + e_pos.z * e_pos.z);
                        float velErrorSq = (float)(e_vel.x * e_vel.x + e_vel.y * e_vel.y + e_vel.z * e_vel.z);
                        payloadGain += gammaPayload * tensionError * dt; payloadGain = Math.Max(0f, payloadGain);
                        Kp += gammaKp * posErrorSq * dt; Kp = Math.Max(0.1f, Kp);
                        Kv += gammaKv * velErrorSq * dt; Kv = Math.Max(0.1f, Kv);

                        // หันตาม leader เมื่อ hover
                        double psi_ff  = leaderHover ? Leader.cs.yaw : (Leader.cs.yaw + yawR * dt);
                        double psi_err = wrap_180(psi_ff - mav.cs.yaw);

                        var quat = Quaternion.from_euler312(
                            (float)(phi * Math.PI / 180.0),
                            (float)(tht * Math.PI / 180.0),
                            (float)(psi_err * Math.PI / 180.0));

                        var msg = new MAVLink.mavlink_set_attitude_target_t
                        {
                            target_system = mav.sysid,
                            target_component = mav.compid,
                            type_mask = 0b10000101 | 0b01000000,
                            q = new float[] { (float)quat.q1, (float)quat.q2, (float)quat.q3, (float)quat.q4 },
                            thrust = thrust
                        };
                        port.sendPacket(msg, mav.sysid, mav.compid);
                        lastUpdateTime = now;
                    }
                    else
                    {
                        // ===== ฝั่ง Copter/อื่น ๆ =====
                        if (leaderHover)
                        {
                            // ตอน hover: บังคับ yaw ให้ตรงกับ leader
                            var vzero = new Vector3(0f, 0f, 0f);
                            port.setPositionTargetGlobalInt(
                                mav.sysid, mav.compid,
                                true,  true,  false, true, // useYaw = true
                                MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT_INT,
                                targetGeo.Lat, targetGeo.Lng, targetGeo.Alt,
                                vzero.x, vzero.y, vzero.z,
                                leaderYawRad, // yaw (radians)
                                0f            // yaw rate
                            );
                        }
                        else
                        {
                            // ปกติ: บินตามตำแหน่งและความเร็ว leader (ไม่บังคับ yaw)
                            var vff = new Vector3((float)Leader.cs.vx, (float)Leader.cs.vy, (float)Leader.cs.vz);
                            port.setPositionTargetGlobalInt(
                                mav.sysid, mav.compid,
                                true,  true,  false, false, // useYaw = false
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
    }

    /// <summary>PID controller</summary>
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
    }

    /// <summary>Vector utilities</summary>
    internal static class VectorUtils
    {
        public static Vector3 NormalizeVector(Vector3 v)
        {
            double len = Math.Sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
            return len == 0 ? Vector3.Zero : new Vector3((float)(v.x / len), (float)(v.y / len), (float)(v.z / len));
        }
    }

    /// <summary>Payload wrench solver with null-space tension balancing</summary>
    internal class LoadAttitudeController
    {
        public double PayloadMass { get; set; } = 1.0;
        private static readonly Matrix<double> JL = Matrix<double>.Build.DenseOfArray(new double[,] {
            {0.021, 0, 0}, {0, 0.0187, 0}, {0, 0, 0.0397}
        });
        private static readonly Dictionary<int, Tuple<float,float>> YawHistory = new Dictionary<int, Tuple<float,float>>();

        public float EstimateYawRate(MAVState leader)
        {
            int id = leader.sysid;
            float cy = leader.cs.yaw;
            float ct = (float)(DateTime.UtcNow - new DateTime(1970,1,1)).TotalSeconds;
            if (YawHistory.ContainsKey(id))
            {
                var (py, pt) = YawHistory[id];
                float dy = (float)Wrap(cy - py);
                float dt = ct - pt;
                YawHistory[id] = Tuple.Create(cy, ct);
                return dt>1e-3f?dy/dt:0f;
            }
            YawHistory[id] = Tuple.Create(cy, ct);
            return 0f;
        }

        public Vector3 CompensateRigidBodyDynamics(MAVState leader, MAVState follower, Dictionary<MAVState, Vector3> offsets)
        {
            if (!offsets.ContainsKey(follower)) return Vector3.Zero;
            var pts = new List<Vector3>(offsets.Values);
            if (pts.Count < 3) return Vector3.Zero;
            var rOff = offsets[follower];
            float yawRate = EstimateYawRate(leader);
            float compTc = Math.Abs(yawRate)>10?0.05f:0.2f;
            float dt = 0.05f;
            float alpha = dt/(compTc+dt);
            float m = (float)PayloadMass;

            var accel = Vector<double>.Build.DenseOfArray(new double[]{
                leader.cs.ax*m, leader.cs.ay*m, (leader.cs.az+9.81f)*m
            });
            var grav  = Vector<double>.Build.Dense(new[]{0.0,0.0,-9.81})*m;
            var R     = BuildRotationMatrix(leader.cs.roll,leader.cs.pitch,leader.cs.yaw);
            var Wf    = -R.Transpose()*(accel+grav);

            var omega   = Vector<double>.Build.Dense(3);
            var omegaDot= Vector<double>.Build.Dense(3);
            var Wm      = -(JL*omegaDot + Cross(omega, JL*omega));

            var W = Vector<double>.Build.Dense(6);
            W.SetSubVector(0,3,Wf);
            W.SetSubVector(3,3,Wm);

            var Phi = Matrix<double>.Build.Dense(6, pts.Count);
            for(int i=0;i<pts.Count;i++)
            {
                var r=pts[i]; var qi=VectorUtils.NormalizeVector(r);
                Phi[0,i]=qi.x; Phi[1,i]=qi.y; Phi[2,i]=qi.z;
                Phi[3,i]=r.y*qi.z-r.z*qi.y;
                Phi[4,i]=r.z*qi.x-r.x*qi.z;
                Phi[5,i]=r.x*qi.y-r.y*qi.x;
            }

            var pinv  = Phi.PseudoInverse();
            var Tpart = pinv * W;
            var N     = Matrix<double>.Build.DenseIdentity(pts.Count) - pinv * Phi;

            double avgT = Tpart.Sum()/pts.Count;
            var equal   = Vector<double>.Build.Dense(pts.Count, avgT);
            var Tnull   = N * equal;
            var Tfinal  = Tpart + Tnull;

            int idx = new List<MAVState>(offsets.Keys).IndexOf(follower);
            if (idx>=0 && idx<Tfinal.Count)
            {
                var qi = VectorUtils.NormalizeVector(rOff);
                float T = (float)Math.Max(0, Tfinal[idx]) * alpha;
                return new Vector3(qi.x * T, qi.y * T, qi.z * T);
            }
            return Vector3.Zero;
        }

        private static double Wrap(double a)
        {
            if (a > 180) return a - 360;
            if (a < -180) return a + 360;
            return a;
        }

        private static Matrix<double> BuildRotationMatrix(float roll,float pitch,float yaw)
        {
            double r=roll*(Math.PI/180.0), p=pitch*(Math.PI/180.0), y=yaw*(Math.PI/180.0);
            double cr=Math.Cos(r), sr=Math.Sin(r), cp=Math.Cos(p), sp=Math.Sin(p), cy=Math.Cos(y), sy=Math.Sin(y);
            return Matrix<double>.Build.DenseOfArray(new double[,]{
                {cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr},
                {sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr},
                {-sp,   cp*sr,            cp*cr}
            });
        }

        private static Vector<double> Cross(Vector<double>a, Vector<double>b)
            => Vector<double>.Build.DenseOfArray(new double[]{
                a[1]*b[2]-a[2]*b[1],
                a[2]*b[0]-a[0]*b[2],
                a[0]*b[1]-a[1]*b[0]
            });
    }
}
