using System;

namespace CsCore
{
    namespace Data
    {
        namespace CollisionProcessor
        {
            public class CollisionRequestZone
            {
                double timestamp;
                int zone;

                public CollisionRequestZone(double timestamp, int zone)
                {
                    this.timestamp = timestamp;
                    this.zone = zone;
                }
                public int GetBytes(ref byte[] buffer, int offset, int length)
                {
                    int size = 0;
                    if (length >= 12)
                    {
                        byte[] byteTimestamp = BitConverter.GetBytes(timestamp);
                        byte[] byteZone = BitConverter.GetBytes(zone);
                        Array.Copy(byteTimestamp, 0, buffer, offset, 8);
                        Array.Copy(byteZone, 0, buffer, offset+8, 4);
                        size = 12;
                    }
                    return size;
                }
            }
        }
    }
}