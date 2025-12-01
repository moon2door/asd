using System;

namespace CsCore
{
    namespace Data
    {
        namespace CollisionProcessor
        {
            public class StMaxDistance
            {
                public static int type = 0x02;
                public static int code = 0x31;
                
                public double maxDistance;

                public StMaxDistance()
                {
                }

                public StMaxDistance(byte[] buffer)
                {
                    FromByte(buffer);
                    
                }
                public void FromByte(byte[] buffer)
                {
                    if (buffer.Length >= 8)
                    {
                        maxDistance = BitConverter.ToDouble(buffer, 0);
                    }
                }

                public int GetBytes(ref byte[] buffer, int offset, int length)
                {
                    var size = 0;
                    if (length < 8) return size;
                    var byteMaxDistance = BitConverter.GetBytes(maxDistance);
                    Array.Copy(byteMaxDistance, 0, buffer, offset, 8);
                    size = 8;
                    return size;
                }
            }
        }
    }
}