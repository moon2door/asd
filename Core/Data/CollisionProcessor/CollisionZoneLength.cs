using System;

namespace CsCore
{
    namespace Data
    {
        namespace CollisionProcessor
        {
            public class CollisionZoneLength
            {
                static public int type { get; private set; } = 0x01;
                static public int code { get; private set; } = 0x13;

                public double timestamp;
                public int zoneNum;
                public float zone1;
                public float zone2;
                public float zone3;
                public float zone4;

                public CollisionZoneLength()
                {
                    timestamp = 0;
                    zoneNum = 0;
                    zone1 = 0;
                    zone2 = 0;
                    zone3 = 0;
                    zone4 = 0;
                }

                public CollisionZoneLength(byte[] buffer)
                {
                    FromByte(buffer);
                }

                public void FromByte(byte[] buffer)
                {
                    if (buffer.Length >= 25)
                    {
                        timestamp = BitConverter.ToDouble(buffer, 0);
                        zoneNum = (int)buffer[8];
                        zone1 = BitConverter.ToSingle(buffer, 9);
                        zone2 = BitConverter.ToSingle(buffer, 13);
                        zone3 = BitConverter.ToSingle(buffer, 17);
                        zone4 = BitConverter.ToSingle(buffer, 21);

                    }
                }

                public int GetBytes(ref byte[] buffer, int offset, int length)
                {
                    int size = 0;
                    if (length >= 25)
                    {
                        byte[] byteTimestamp = BitConverter.GetBytes(timestamp);
                        byte[] byteZone1 = BitConverter.GetBytes(zone1);
                        byte[] byteZone2 = BitConverter.GetBytes(zone2);
                        byte[] byteZone3 = BitConverter.GetBytes(zone3);
                        byte[] byteZone4 = BitConverter.GetBytes(zone4);
                        Array.Copy(byteTimestamp, 0, buffer, offset, 8);
                        buffer[offset+8] = (byte)zoneNum;
                        Array.Copy(byteZone1, 0, buffer, offset+9, 4);
                        Array.Copy(byteZone2, 0, buffer, offset+13, 4);
                        Array.Copy(byteZone3, 0, buffer, offset+17, 4);
                        Array.Copy(byteZone4, 0, buffer, offset + 21, 4);
                        size = 25;
                    }
                    return size;
                }
            }
        }
    }
}