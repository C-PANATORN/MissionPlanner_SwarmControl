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
    /// <summary>
    /// Formation control with analytic flatness feed-forward, unified feedback,
    /// full wrench-based tension distribution, and yaw feed-forward (§3.1–3.4).
    /// </summary>
    internal class Formation : Swarm
    {
        private readonly Dictionary<MAVState, Vector3> offsets = new Dictionary<MAVState, Vector3>();
        private readonly Dictionary<MAVState, Vector3> compFiltered = new Dictionary<MAVState, Vector3>();
        private readonly Dictionary<MAVState, float> prevYawRate = new Dictionary<MAVState, float>();
        private readonly LoadAttitudeController payloadController = new LoadAttitudeController();

        private const float payloadGain = 0.5f;
        private const float compTau     = 0.2f;
        private const float Kp = 1.0f;
        private const float Kv = 0.5f;

        private readonly CoordinateTransformationFactory ctfac = new CoordinateTransformationFactory();
        private readonly IGeographicCoordinateSystem wgs84 = GeographicCoordinateSystem.WGS84;

        private PointLatLngAlt masterpos;
        private DateTime lastUpdateTime = DateTime.UtcNow;

        public void setOffsets(MAVState mav, double x, double y, double z) =>
            offsets[mav] = new Vector3((float)x, (float)y, (float)z);

        public Vector3 getOffsets(MAVState mav) =>
            offsets.ContainsKey(mav) ? offsets[mav] : Vector3.Zero;

        public override void Update()
        {
            var mavBase = MainV2.comPort.MAV;
            if (mavBase.cs.lat == 0 || mavBase.cs.lng == 0) return;
            if (Leader == null) Leader = mavBase;
            masterpos = new PointLatLngAlt(Leader.cs.lat, Leader.cs.lng, Leader.cs.alt, "");
            lastUpdateTime = DateTime.UtcNow;
        }

        private double wrap_180(double a)
        {
            if (a > 180) return a - 360;
            if (a < -180) return a + 360;
            return a;
        }

        public override void SendCommand()
        {
            if (Leader == null || masterpos.Lat == 0) return;

            int zone = (int)((masterpos.Lng + 180.0) / 6.0);
            var utm = ProjectedCoordinateSystem.WGS84_UTM(zone, masterpos.Lat >= 0);
            var trans = ctfac.CreateFromCoordinateSystems(wgs84, utm);
            double[] leadXY = trans.MathTransform.Transform(new double[] { masterpos.Lng, masterpos.Lat });
            Vector3 leaderUTM = new Vector3((float)leadXY[0], (float)leadXY[1], (float)masterpos.Alt);

            foreach (var port in MainV2.Comports)
            {
                foreach (var mav in port.MAVlist)
                {
                    if (mav == Leader) continue;

                    // Desired target
                    Vector3 off = getOffsets(mav);
                    double hdg = -Leader.cs.yaw * (Math.PI / 180.0);
                    Vector3 targetUTM = new Vector3(
                        leaderUTM.x + off.x * (float)Math.Cos(hdg) - off.y * (float)Math.Sin(hdg),
                        leaderUTM.y + off.x * (float)Math.Sin(hdg) + off.y * (float)Math.Cos(hdg),
                        (float)masterpos.Alt + off.z
                    );
                    double[] inv = trans.MathTransform.Inverse().Transform(new double[] { targetUTM.x, targetUTM.y });
                    var targetGeo = new PointLatLngAlt(inv[1], inv[0], targetUTM.z, "");

                    try
                    {
                        if (mav.cs.firmware == Firmwares.ArduPlane)
                        {
                            DateTime now = DateTime.UtcNow;
                            double dt = (now - lastUpdateTime).TotalSeconds;

                            // 1) Analytic feed-forward accel (flatness) §3.2
                            Vector3 aL = new Vector3(Leader.cs.ax, Leader.cs.ay, Leader.cs.az);
                            float yawRate = payloadController.EstimateYawRate(Leader);
                            float yawAcc = prevYawRate.TryGetValue(mav, out var prev) ? (yawRate - prev) / (float)dt : 0f;
                            prevYawRate[mav] = yawRate;
                            Vector3 a_cent = new Vector3(
                                -yawRate * yawRate * off.x,
                                -yawRate * yawRate * off.y,
                                0f);
                            Vector3 a_tan = new Vector3(
                                -off.y * yawAcc,
                                 off.x * yawAcc,
                                0f);
                            Vector3 accel_ff = aL + a_cent + a_tan;

                            // 2) Feedback accel on formation error §3.3
                            double[] curXY = trans.MathTransform.Transform(new double[] { mav.cs.lng, mav.cs.lat });
                            Vector3 curUTM = new Vector3((float)curXY[0], (float)curXY[1], mav.cs.alt);
                            Vector3 e_pos = targetUTM - curUTM;
                            Vector3 curVel = new Vector3(mav.cs.vx, mav.cs.vy, mav.cs.vz);
                            Vector3 vel_ff = new Vector3(Leader.cs.vx, Leader.cs.vy, Leader.cs.vz) +
                                new Vector3(-yawRate * off.y, yawRate * off.x, 0f);
                            Vector3 e_vel = vel_ff - curVel;
                            Vector3 accel_fb = e_pos * Kp + e_vel * Kv;

                            // 3) Wrench-based tension comp §3.1–3.4
                            Vector3 payloadComp = payloadController.CompensateRigidBodyDynamics(Leader, mav, offsets);
                            float alpha = (float)(dt / (compTau + dt));
                            Vector3 prevC = compFiltered.ContainsKey(mav) ? compFiltered[mav] : Vector3.Zero;
                            Vector3 smoothComp = prevC + (payloadComp - prevC) * alpha;
                            compFiltered[mav] = smoothComp;

                            // 4) Total accel
                            Vector3 u = accel_ff + accel_fb + smoothComp * payloadGain;

                            // 5) Attitude + thrust inversion §3.4
                            double phi = Math.Asin(Math.Max(-1, Math.Min(1, u.y / 9.81))) * (180.0 / Math.PI);
                            double theta = Math.Asin(Math.Max(-1, Math.Min(1, -u.x / 9.81))) * (180.0 / Math.PI);
                            float thrust = (float)Math.Max(0.1, Math.Min(1, (u.z + 9.81) / 9.81));

                            // 6) Yaw feed-forward + feedback
                            double psi_ff = Leader.cs.yaw;
                            double psi_err = wrap_180(psi_ff - mav.cs.yaw);

                            var quat = Quaternion.from_euler312(
                                (float)(phi * Math.PI / 180.0),
                                (float)(theta * Math.PI / 180.0),
                                (float)(psi_err * Math.PI / 180.0));
                            var msg = new MAVLink.mavlink_set_attitude_target_t
                            {
                                target_system = mav.sysid,
                                target_component = mav.compid,
                                type_mask = 0b10000101 ^ 0b01000000,
                                q = new float[] { (float)quat.q1, (float)quat.q2, (float)quat.q3, (float)quat.q4 },
                                thrust = thrust
                            };
                            port.sendPacket(msg, mav.sysid, mav.compid);
                        }
                        else
                        {
                            var v = new Vector3(Leader.cs.vx, Leader.cs.vy, Leader.cs.vz);
                            port.setPositionTargetGlobalInt(mav.sysid, mav.compid,
                                true, true, false, false,
                                MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT_INT,
                                targetGeo.Lat, targetGeo.Lng, targetGeo.Alt,
                                v.x, v.y, v.z, 0, 0);
                        }
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine($"SendCommand error {mav.sysid}: {ex}");
                    }
                }
            }
            lastUpdateTime = DateTime.UtcNow;
        }
    }

    /// <summary>Simple PID controller</summary>
    internal class PID
    {
        private float dt, input, deriv, integrator, imax, filtHz;
        private readonly float kp, ki, kd;
        private bool resetFilter = true;

        public PID(float p,float i,float d,float im, float fHz, float dt0, float ff)
        {
            kp = p; ki = i; kd = d; imax = Math.Abs(im);
            filtHz = Math.Max(fHz, 0.01f); dt = dt0;
        }
        public void set_input_filter_all(float inVal)
        {
            if(float.IsInfinity(inVal)) return;
            if(resetFilter){input=inVal;deriv=0;resetFilter=false;}
            float alpha = dt/(dt + 1f/(2f*(float)Math.PI*filtHz));
            float delta = alpha*(inVal-input);
            input += delta;
            deriv = dt>0?delta/dt:0;
        }
        public float get_pid()
        {
            float P = kp*input;
            integrator = Math.Max(-imax, Math.Min(imax, integrator+ki*input*dt));
            float D = kd*deriv;
            return P + integrator + D;
        }
    }

    /// <summary>Vector utils</summary>
    internal static class VectorUtils
    {
        public static Vector3 NormalizeVector(Vector3 v)
        {
            double len = Math.Sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
            return len==0?Vector3.Zero:new Vector3((float)(v.x/len),(float)(v.y/len),(float)(v.z/len));
        }
    }

    /// <summary>Payload wrench solver</summary>
    internal class LoadAttitudeController
    {
        public double PayloadMass { get; set; } = 1.0;
        private static readonly Matrix<double> JL = Matrix<double>.Build.DenseOfArray(new double[,]{{0.021,0,0},{0,0.0187,0},{0,0,0.0397}});
        private static readonly Dictionary<int, Tuple<float,float>> YawHistory = new Dictionary<int, Tuple<float,float>>();

        public float EstimateYawRate(MAVState leader)
        {
            int id=leader.sysid;
            float currentYaw=leader.cs.yaw;
            float currentTime=(float)(DateTime.UtcNow-new DateTime(1970,1,1)).TotalSeconds;
            if(YawHistory.ContainsKey(id)){
                var (pY,pT)=YawHistory[id];
                float dyaw=(float)Wrap(currentYaw-pY);
                float dt=currentTime-pT;
                YawHistory[id]=Tuple.Create(currentYaw,currentTime);
                return dt>1e-3f?dyaw/dt:0f;
            }
            YawHistory[id]=Tuple.Create(currentYaw,currentTime);
            return 0f;
        }

        public Vector3 CompensateRigidBodyDynamics(MAVState leader,MAVState follower,Dictionary<MAVState,Vector3> offsets)
        {
            if(!offsets.ContainsKey(follower))return Vector3.Zero;
            var pts=new List<Vector3>(offsets.Values);
            if(pts.Count<3)return Vector3.Zero;
            var staticOff=offsets[follower];
            float yawRate=EstimateYawRate(leader);
            float compTc=Math.Abs(yawRate)>10?0.05f:0.2f;
            float dt=0.05f; float alpha=dt/(compTc+dt);
            float m=(float)PayloadMass;
            var accel=Vector<double>.Build.DenseOfArray(new double[]{leader.cs.ax*m,leader.cs.ay*m,(leader.cs.az+9.81f)*m});
            var grav=Vector<double>.Build.Dense(new[]{0.0,0.0,-9.81})*m;
            var R=BuildRotationMatrix(leader.cs.roll,leader.cs.pitch,leader.cs.yaw);
            var Wf=-R.Transpose()*(accel+grav);
            var omega=Vector<double>.Build.Dense(3);var omegaDot=Vector<double>.Build.Dense(3);
            var Wm=-(JL*omegaDot+Cross(omega,JL*omega));
            var W=Vector<double>.Build.Dense(6);W.SetSubVector(0,3,Wf);W.SetSubVector(3,3,Wm);
            var Phi=Matrix<double>.Build.Dense(6,pts.Count);
            for(int i=0;i<pts.Count;i++){var r=pts[i];var qi=VectorUtils.NormalizeVector(r);Phi[0,i]=qi.x;Phi[1,i]=qi.y;Phi[2,i]=qi.z;Phi[3,i]=r.y*qi.z-r.z*qi.y;Phi[4,i]=r.z*qi.x-r.x*qi.z;Phi[5,i]=r.x*qi.y-r.y*qi.x;}
            var T=Phi.PseudoInverse()*W;int idx=new List<MAVState>(offsets.Keys).IndexOf(follower);
            if(idx>=0&&idx<T.Count){var qi=VectorUtils.NormalizeVector(staticOff);return qi*(float)Math.Max(0,T[idx])*alpha;}return Vector3.Zero;
        }

        private double Wrap(double a){if(a>180)return a-360; if(a<-180)return a+360; return a;}
        private Matrix<double> BuildRotationMatrix(float roll,float pitch,float yaw){double r=roll*(Math.PI/180.0),p=pitch*(Math.PI/180.0),y=yaw*(Math.PI/180.0);double cr=Math.Cos(r),sr=Math.Sin(r),cp=Math.Cos(p),sp=Math.Sin(p),cy=Math.Cos(y),sy=Math.Sin(y);return Matrix<double>.Build.DenseOfArray(new double[,]{{cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr},{sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr},{-sp,cp*sr,cp*cr}});}        
        private Vector<double> Cross(Vector<double>a,Vector<double>b)=>Vector<double>.Build.DenseOfArray(new double[]{a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]});    }
}
