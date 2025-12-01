using System;
using System.Collections.Generic;
using UnityEngine;

namespace CsCore
{
    namespace Data
    {
        namespace CollisionProcessor
        {
            public class StMonitoringLabelInfo
            {
                public static int type = 0x02;
                public static int code = 0x30;

                public float[] xyz = new float[0];
                public string info = "";

                public StMonitoringLabelInfo()
                {
                }

                public StMonitoringLabelInfo(byte[] buffer)
                {
                    FromByte(buffer);
                }

                public void FromByte(byte[] buffer)
                {
                    if (buffer.Length >= 96)
                    {
                        int numLabel = BitConverter.ToInt32(buffer, 0);
                        if(numLabel <= 5)
                        {
                            xyz = new float[3 * numLabel];
                            for (int i = 0; i < numLabel; i++)
                            {
                                float x = BitConverter.ToSingle(buffer, 4 + 4 * (i * 3 + 0));
                                float y = BitConverter.ToSingle(buffer, 4 + 4 * (i * 3 + 1));
                                float z = BitConverter.ToSingle(buffer, 4 + 4 * (i * 3 + 2));
                                xyz[i * 3 + 0] = x;
                                xyz[i * 3 + 1] = y;
                                xyz[i * 3 + 2] = z;
                            }
                        }
                        
                        info = System.Text.Encoding.UTF8.GetString(buffer, 64, 32);
                    }
                }

                public int GetBytes(ref byte[] buffer, int offset, int length)
                {
                    int size = 0;
                    if (length >= 96)
                    {
                        int count = xyz.Length / 3;
                        if (count > 5) count = 5;
                        byte[] byteNumLabel = BitConverter.GetBytes(count);
                        
                        Array.Copy(byteNumLabel, 0, buffer, offset, 4);
                        for(int i=0; i< count * 3; i+=3)
                        {
                            byte[] x = BitConverter.GetBytes(xyz[i+0]);
                            byte[] y = BitConverter.GetBytes(xyz[i+1]);
                            byte[] z = BitConverter.GetBytes(xyz[i+2]);
                            Array.Copy(x, 0, buffer, offset + 4 + 4 * (i + 0), 4);
                            Array.Copy(y, 0, buffer, offset + 4 + 4 * (i + 1), 4);
                            Array.Copy(z, 0, buffer, offset + 4 + 4 * (i + 2), 4);
                        }
                        byte[] byteInfo = System.Text.Encoding.UTF8.GetBytes(info);
                        int lengthByteInfo = byteInfo.Length;
                        if (lengthByteInfo > 32) lengthByteInfo = 32;
                        Array.Copy(byteInfo, 0, buffer, offset + 4 + 4 * 3 * 5, lengthByteInfo);
                        size = 96;
                    }
                    return size;
                }
            }
        }
    }
}