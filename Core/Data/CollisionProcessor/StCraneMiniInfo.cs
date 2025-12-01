using System;

namespace CsCore
{
    namespace Data
    {
        namespace CollisionProcessor
        {
            public class StCraneMiniInfo
            {
                public static int type = 0x04;
                public static int code = 0x28;

                public int pier;
                public int crane;
                public int collisionTotal;
                public double distance;
                public double maxDistance;

                public StCraneMiniInfo(byte[] buffer)
                {
                    pier = BitConverter.ToInt32(buffer, 0);
                    crane = BitConverter.ToInt32(buffer, 4);
                    collisionTotal = BitConverter.ToInt32(buffer, 824);
                    distance = BitConverter.ToSingle(buffer, 828);
                }
            }
        }
    }
}