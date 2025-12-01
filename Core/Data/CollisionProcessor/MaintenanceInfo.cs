using System;

namespace CsCore
{
    namespace Data
    {
        namespace CollisionProcessor
        {
            public struct MaintenanceInfoSingle
            {
                public string rotorSerial;
                public string lidarSerial;
                public DateTime rotorStartTime;
                public DateTime lidarStartTime;
            }

            public class MaintenanceInfo
            {
                public static int type = 0x01;
                public static int code = 0x0F;

                public MaintenanceInfoSingle[] info = new MaintenanceInfoSingle[5];

                public MaintenanceInfo(byte[] buffer)
                {
                    FromByte(buffer);
                }

                public void FromByte(byte[] buffer)
                {
                    if (buffer.Length >= 112*5)
                    {
                        for(int i=0; i<5; i++)
                        {
                            int offset = i * 214;
                            info[i].rotorSerial = System.Text.Encoding.UTF8.GetString(buffer, offset + 0, offset + 32);
                            info[i].lidarSerial = System.Text.Encoding.UTF8.GetString(buffer, offset + 32, offset + 64);

                            int rotorYear = BitConverter.ToInt32(buffer, offset + 64);
                            int rotorMonth = BitConverter.ToInt32(buffer, offset + 68);
                            int rotorDay = BitConverter.ToInt32(buffer, offset + 72);
                            int rotorHour = BitConverter.ToInt32(buffer, offset + 76);
                            int rotorMinute = BitConverter.ToInt32(buffer, offset + 80);
                            int rotorSecond = BitConverter.ToInt32(buffer, offset + 84);

                            int lidarYear = BitConverter.ToInt32(buffer, offset + 88);
                            int lidarMonth = BitConverter.ToInt32(buffer, offset + 92);
                            int lidarDay = BitConverter.ToInt32(buffer, offset + 96);
                            int lidarHour = BitConverter.ToInt32(buffer, offset + 100);
                            int lidarMinute = BitConverter.ToInt32(buffer, offset + 104);
                            int lidarSecond = BitConverter.ToInt32(buffer, offset + 108);

                            try
                            {
                                info[i].rotorStartTime = new DateTime(rotorYear, rotorMonth, rotorDay, rotorHour, rotorMinute, rotorSecond);
                                info[i].lidarStartTime = new DateTime(lidarYear, lidarMonth, lidarDay, lidarHour, lidarMinute, lidarSecond);
                            }
                            catch(Exception)
                            {

                            }
                        }
                    }
                }
            }
        }
    }
}