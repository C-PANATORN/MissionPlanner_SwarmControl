using log4net;
using System.Reflection;
using System.Threading;
using System.Threading.Tasks; // ★ เพิ่ม
using System;

namespace MissionPlanner.Swarm
{
    abstract class Swarm
    {
        internal static readonly ILog log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);
        internal MAVState Leader = null;

        private const string MODE_GUIDED = "GUIDED";
        private const string MODE_AUTO   = "AUTO";
        private const string MODE_LAND   = "LAND";
        private const string MODE_LOITER = "LOITER";

        public void setLeader(MAVState lead)
        {
            Leader = lead;

            foreach (var port in MainV2.Comports)
            {
                foreach (var mav in port.MAVlist)
                {
                    if (mav == Leader) continue;
                    SetModeSafe(port, mav, MODE_GUIDED);
                }
            }
        }

        public MAVState getLeader() => Leader;

        private void SetModeSafe(dynamic port, MAVState mav, string mode)
        {
            try { port.setMode(mav.sysid, mav.compid, mode); }
            catch (Exception ex) { log.Warn($"SetMode {mode} failed for sysid {mav?.sysid}", ex); }
        }

        // (ยังคงไว้ เผื่อใช้ที่อื่นต่อ)
        private bool DoTakeoffWithRetry(dynamic port, MAVState mav, float alt, int retries = 1, int delayMs = 300)
        {
            for (int i = 0; i <= retries; i++)
            {
                try
                {
                    bool ok = port.doCommand(
                        mav.sysid, mav.compid,
                        MAVLink.MAV_CMD.TAKEOFF,
                        0, 0, 0, 0, 0, 0, alt
                    );
                    if (ok) return true;
                }
                catch (Exception ex)
                {
                    log.Warn($"TAKEOFF failed for sysid {mav?.sysid} (try {i + 1})", ex);
                }

                if (i < retries) Thread.Sleep(delayMs);
            }
            return false;
        }

        public void Arm()
        {
            foreach (var port in MainV2.Comports)
            {
                try { port.doARM(true, true); }
                catch (Exception ex) { log.Warn("Force ARM failed on a port", ex); }
            }
        }

        public void Disarm()
        {
            foreach (var port in MainV2.Comports)
            {
                try { port.doARM(false, false); }
                catch (Exception ex) { log.Warn("DISARM failed on a port", ex); }
            }
        }

        // ★★ แก้ให้ยิงคำสั่งแบบ background + parallel และ return ทันที ★★
        public void Takeoff(float altitude)
        {
            if (float.IsNaN(altitude) || altitude <= 0f) altitude = 0.5f;

            _ = Task.Run(() =>
            {
                Parallel.ForEach(MainV2.Comports, port =>
                {
                    Parallel.ForEach(port.MAVlist, mav =>
                    {
                        try
                        {
                            // สไตล์ที่คุณต้องการ: ตั้ง GUIDED แล้วสั่ง TAKEOFF ตรง ๆ
                            port.setMode(mav.sysid, mav.compid, MODE_GUIDED);
                            port.doCommand(
                                mav.sysid, mav.compid,
                                MAVLink.MAV_CMD.TAKEOFF,
                                0, 0, 0, 0, 0, 0, altitude
                            );
                        }
                        catch
                        {
                            // เงียบไว้ตามต้องการ (หลีกเลี่ยงการหยุดทั้งฝูง)
                        }
                    });
                });
            });
        }

        public void Land()
        {
            foreach (var port in MainV2.Comports)
            {
                foreach (var mav in port.MAVlist)
                {
                    SetModeSafe(port, mav, MODE_LAND);
                }
            }
        }

        public void Stop()
        {
            // ไว้ต่อยอดภายหลังถ้าต้องการ
        }

        public void GuidedMode()
        {
            foreach (var port in MainV2.Comports)
            {
                foreach (var mav in port.MAVlist)
                {
                    SetModeSafe(port, mav, MODE_GUIDED);
                }
            }
        }

        public void AutoMode() //Loiter
        {
            if (Leader == null) return;

            // สั่งโหมด LOITER เฉพาะลำ Leader เท่านั้น
            foreach (var port in MainV2.Comports)
            {
                foreach (var mav in port.MAVlist)
                {
                    if (mav == Leader)
                    {
                        SetModeSafe(port, mav, MODE_LOITER);
                        return; // เจอ Leader แล้วก็จบ ไม่ยุ่งลำอื่น
                    }
                }
            }
        }


        public abstract void Update();
        public abstract void SendCommand();
    }
}
