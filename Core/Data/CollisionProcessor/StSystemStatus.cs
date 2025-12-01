using System;

namespace CsCore
{
    namespace Data
    {
        namespace CollisionProcessor
        {
            public class StSystemStatus
            {
                public static int type = 0x01;
                public static int code = 0x0c;
                public double timestamp;
                public bool networkSensorCom;
                public bool networkDBMS;
                public bool networkPLC;
                public bool networkGPS;
                public bool networkRouter;
                public bool networkGPSData;
                public bool bUseAlarm;

                public uint versionInterface;
                public uint versionCluster;
                public uint versionDistance;
                public uint versionCollision;
                public uint versionManagerSensor;
                public uint versionManagerCollision;

                public float windSpeed;
                public float windDirection;
                public int rpm;

                public StRotorInfo[] rotorStatus;

                public StSystemStatus()
                {
                    rotorStatus = new StRotorInfo[2];
                }

                public StSystemStatus(byte[] buffer)
                {
                    FromByte(buffer);
                }

                public StSystemStatus Clone()
                {
                    StSystemStatus clone = (StSystemStatus)this.MemberwiseClone();
                    if (rotorStatus != null)
                    {
                        clone.rotorStatus = new StRotorInfo[rotorStatus.Length];
                        for (int i = 0; i < rotorStatus.Length; i++)
                        {
                            clone.rotorStatus[i] = rotorStatus[i];
                        }
                    }
                    return clone;
                }
                public void FromByte(byte[] buffer)
                {
                    if (buffer.Length >= 6)
                    {
                        timestamp = BitConverter.ToDouble(buffer, 0);

                        byte statusNetwork = buffer[8];
                        networkSensorCom = (((statusNetwork >> 0) & 0x01) == 0x01);
                        networkDBMS = (((statusNetwork >> 1) & 0x01) == 0x01);
                        networkPLC = (((statusNetwork >> 2) & 0x01) == 0x01);
                        networkGPS = (((statusNetwork >> 3) & 0x01) == 0x01);
                        networkRouter = (((statusNetwork >> 4) & 0x01) == 0x01);
                        networkGPSData = (((statusNetwork >> 5) & 0x01) == 0x01);

                        bool statusCooprtationMode = (buffer[9] == 1);
                        bUseAlarm = (buffer[10] == 1);
                        bool bModeEmergencyStop = (buffer[11] == 1);

                        versionInterface = BitConverter.ToUInt32(buffer, 12);
                        versionCluster = BitConverter.ToUInt32(buffer, 16);
                        versionDistance = BitConverter.ToUInt32(buffer, 20);
                        versionCollision = BitConverter.ToUInt32(buffer, 24);
                        versionManagerSensor = BitConverter.ToUInt32(buffer, 28);
                        versionManagerCollision = BitConverter.ToUInt32(buffer, 32);

                        windSpeed = BitConverter.ToSingle(buffer, 36);
                        windDirection = BitConverter.ToSingle(buffer, 40);

                        float reserved1 = BitConverter.ToSingle(buffer, 44);
                        float reserved2 = BitConverter.ToSingle(buffer, 48);

                        int craneType = (int)buffer[52];

                        int statusVinfoPos = BitConverter.ToInt32(buffer, 53);
                        int statusVinfoSize = BitConverter.ToInt32(buffer, 57);
                        int vSize = BitConverter.ToInt32(buffer, 61);

                        int numRotor = statusVinfoSize / 24;
                        rotorStatus = new StRotorInfo[numRotor];
                        int idx = 0;
                        for (int vpos = 65 + statusVinfoPos; vpos < 65 + statusVinfoPos + statusVinfoSize; vpos += 24)
                        {
                            rotorStatus[idx].lidarFps = (int)BitConverter.ToInt16(buffer, vpos);
                            float tempTop = BitConverter.ToSingle(buffer, vpos + 2);
                            float tempBottom = BitConverter.ToSingle(buffer, vpos + 6);
                            byte byteStatus = buffer[vpos + 10];
                            bool networkStatus = (((byteStatus >> 0) & 0x01) == 0x01);
                            bool networkRangeData = (((byteStatus >> 1) & 0x01) == 0x01);
                            byte errorCodeLidar = buffer[vpos + 11];

                            rotorStatus[idx].rotorRpm = BitConverter.ToSingle(buffer, vpos + 12);
                            rotorStatus[idx].rotorFps = BitConverter.ToUInt16(buffer, vpos + 20);

                            byte byteSensor = buffer[vpos + 22];
                            rotorStatus[idx].packetError = (((byteSensor >> 0) & 0x01) == 0);
                            rotorStatus[idx].rotationError = (((byteSensor >> 1) & 0x01) == 0);
                            rotorStatus[idx].proxyMeterError = (((byteSensor >> 2) & 0x01) == 0);
                            rotorStatus[idx].zeroSetError = (((byteSensor >> 3) & 0x01) == 0);
                            rotorStatus[idx].encoderError = (((byteSensor >> 4) & 0x01) == 0);
                            rotorStatus[idx].NetworkError = (((byteSensor >> 5) & 0x01) == 0);
                            rotorStatus[idx].errorCodeActurator = buffer[vpos + 23];
                            idx++;
                        }
                    }
                }
            }
        }
    }
}