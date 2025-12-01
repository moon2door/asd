using System;

namespace CsCore
{
    namespace Data
    {
        namespace CollisionProcessor
        {
            public class StDistanceSocketCompressed
            {
                public static int type = 0x01;
                public static int code = 0x0e;

                public StDistanceSocketCompressed(byte[] buffer)
                {
                    this.buffer = buffer;
                }

                public byte[] buffer;
            }
        }
    }
}