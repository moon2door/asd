using System;

namespace CsCore
{
    namespace Data
    {
        namespace CollisionProcessor
        {
            public class RotorControl
            {
                public static int type = 0x02;
                public static int code = 0x01;
                public byte numSensor;
                public bool bStart;

                public RotorControl()
                {

                }

                public RotorControl(byte[] buffer)
                {
                    FromByte(buffer);
                }
                public void FromByte(byte[] buffer)
                {
                    if (buffer.Length >= 3)
                    {
                        numSensor = buffer[0];
                        if (buffer[2] > 0)
                        {
                            bStart = true;
                        }
                        else
                        {
                            bStart = false;
                        }
                    }
                }

                public int GetBytes(ref byte[] buffer, int offset, int length)
                {
                    int size = 0;
                    if (length >= 3)
                    {
                        buffer[offset] = numSensor;
                        if(bStart)
                        {
                            buffer[offset + 2] = 1;
                        }
                        else
                        {
                            buffer[offset + 2] = 0;
                        }
                        size = 3;
                    }
                    return size;
                }
            }
        }
    }
}