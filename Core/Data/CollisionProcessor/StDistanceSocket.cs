using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.PlayerLoop;

namespace CsCore
{
    namespace Data
    {
        namespace CollisionProcessor
        {
            public class StDistanceSocket
            {
                public static int type { get; private set;} = 0x01;
                public static int code { get; private set; } = 0x04;

                public double timestamp = 0;
                public StCraneAttitude attitude = new StCraneAttitude();
                public float[] xyz = new float[0];
                public StDistance[] distanceInfo = new StDistance[0];
                public List<int> distanceNormal = new List<int>();
                public List<int> distanceHook = new List<int>();
                public List<int> distanceModel = new List<int>();

                public int collisionTotal;
                public byte[] collisionInfo = new byte[0];
                public byte[] reasonInfo = new byte[0];
                public List<int> indicesObjects = new List<int>();
                public List<int> indicesHook = new List<int>();
                public List<int> indicesBody = new List<int>();
                public List<int> indicesGround = new List<int>();
                public List<int> indicesException = new List<int>();
                public StDistanceSocket(byte[] buffer)
                {
                    FromByte(buffer);
                }

                public void FromByte(byte[] buffer)
                {
                    if (buffer.Length >= 921)
                    {
                        timestamp = BitConverter.ToDouble(buffer, 0);

                        byte[] bufferAttitude = new byte[824];
                        Array.Copy(buffer, 16, bufferAttitude, 0, 824);
                        attitude.FromByte(bufferAttitude);

                        int posPoints = BitConverter.ToInt32(buffer, 848);
                        int sizePoints = BitConverter.ToInt32(buffer, 852);

                        int posClusterInfo = BitConverter.ToInt32(buffer, 856);
                        int sizeClusterInfo = BitConverter.ToInt32(buffer, 860);

                        int posClusterIndices = BitConverter.ToInt32(buffer, 864);
                        int sizeClusterIndices = BitConverter.ToInt32(buffer, 868);

                        int posLabel = BitConverter.ToInt32(buffer, 872);
                        int sizeLabel = BitConverter.ToInt32(buffer, 876);

                        int posDistanceInfo = BitConverter.ToInt32(buffer, 880);
                        int sizeDistanceInfo = BitConverter.ToInt32(buffer, 884);

                        int posDistanceLabel = BitConverter.ToInt32(buffer, 888);
                        int sizeDistanceLabel = BitConverter.ToInt32(buffer, 892);

                        uint elapsedTimeCollision = BitConverter.ToUInt32(buffer, 896);

                        collisionTotal = (int)buffer[900];

                        int posCollision = BitConverter.ToInt32(buffer, 901);
                        int numCollision = BitConverter.ToInt32(buffer, 905);

                        int posReasonInfo = BitConverter.ToInt32(buffer, 909);
                        int numReasonInfo = BitConverter.ToInt32(buffer, 913);

                        int posVInfo = 921;
                        xyz = new float[sizePoints * 3 / 6];
                        int idx = 0;
                        for (int pos = posVInfo + posPoints; pos < posVInfo + posPoints + sizePoints; pos += 6)
                        {
                            float x = BitConverter.ToInt16(buffer, pos) * 0.01f;
                            float y = BitConverter.ToInt16(buffer, pos + 2) * 0.01f;
                            float z = BitConverter.ToInt16(buffer, pos + 4) * 0.01f;
                            xyz[idx] = x;
                            xyz[idx + 1] = y;
                            xyz[idx + 2] = z;
                            idx += 3;
                        }
                        
                        int[] clusterInfoOffset = new int[sizeClusterInfo / 8];
                        int[] clusterInfoSize = new int[sizeClusterInfo / 8];
                        idx = 0;
                        for (int pos = posVInfo + posClusterInfo; pos < posVInfo + posClusterInfo + sizeClusterInfo; pos += 8)
                        {
                            int offset = BitConverter.ToInt32(buffer, pos);
                            int size = BitConverter.ToInt32(buffer, pos + 4);

                            clusterInfoOffset[idx] = offset;
                            clusterInfoSize[idx] = size;
                            idx ++;
                        }

                        int[] clusterIndices = new int[sizeClusterIndices / 4];
                        idx = 0;
                        for (int pos = posVInfo + posClusterIndices; pos < posVInfo + posClusterIndices + sizeClusterIndices; pos += 4)
                        {
                            clusterIndices[idx] = BitConverter.ToInt32(buffer, pos);
                            idx++;
                        }

                        int[] clusterLabels = new int[sizeLabel];
                        idx = 0;
                        indicesObjects.Clear();
                        indicesHook.Clear();
                        indicesBody.Clear();
                        indicesGround.Clear();
                        indicesException.Clear();
                        for (int pos = posVInfo + posLabel; pos < posVInfo + posLabel + sizeLabel; pos ++)
                        {
                            clusterLabels[idx] = (int)buffer[pos];

                            const int LABEL_OBJECT = 0;
                            const int LABEL_HOOK = 1;
                            const int LABEL_BODY = 2;
                            const int LABEL_GROUND = 3;
                            const int LABEL_EXCEPTION = 4;

                            int offset = clusterInfoOffset[idx];
                            int size = clusterInfoSize[idx];
                            switch (clusterLabels[idx])
                            {
                                case LABEL_OBJECT:
                                    for(int i=0; i<size; i++)
                                    {
                                        indicesObjects.Add(clusterIndices[offset + i]);
                                    }
                                    break;
                                case LABEL_HOOK:
                                    for (int i = 0; i < size; i++)
                                    {
                                        indicesHook.Add(clusterIndices[offset + i]);
                                    }
                                    break;
                                case LABEL_BODY:
                                    for (int i = 0; i < size; i++)
                                    {
                                        indicesBody.Add(clusterIndices[offset + i]);
                                    }
                                    break;
                                case LABEL_GROUND:
                                    for (int i = 0; i < size; i++)
                                    {
                                        indicesGround.Add(clusterIndices[offset + i]);
                                    }
                                    break;
                                case LABEL_EXCEPTION:
                                    for (int i = 0; i < size; i++)
                                    {
                                        indicesException.Add(clusterIndices[offset + i]);
                                    }
                                    break;
                                default:
                                    break;
                            }
                            idx++;
                        }

                        int[] distanceLabels = new int[sizeDistanceLabel];
                        idx = 0;
                        distanceNormal.Clear();
                        distanceHook.Clear();
                        distanceModel.Clear();
                        for (int pos = posVInfo + posDistanceLabel; pos < posVInfo + posDistanceLabel + sizeDistanceLabel; pos++)
                        {
                            const int DISTANCE_NORMAL = 0;
                            //const int DISTANCE_EXCEPTION = 10;
                            //const int DISTANCE_EXCEPTION_PART = 11;
                            //const int DISTANCE_EXCEPTION_PARTONLY = 12;
                            //const int DISTANCE_EXCEPTION_TRIANGULAR = 13;
                            //const int DISTANCE_EXCEPTION_OUTOFRANGE = 14;
                            const int DISTANCE_HOOK = 1;
                            //const int DISTANCE_HOOK_EXCEPTION = 20;
                            //const int DISTANCE_HOOK_EXCEPTION_PART = 21;
                            //const int DISTANCE_HOOK_EXCEPTION_PARTONLY = 22;
                            //const int DISTANCE_HOOK_EXCEPTION_TRIANGULAR = 23;
                            //const int DISTANCE_HOOK_EXCEPTION_OUTOFRANGE = 24;
                            const int DISTANCE_MODEL = 30;

                            distanceLabels[idx] = (int)buffer[pos];
                            if(distanceLabels[idx] == DISTANCE_NORMAL)
                            {
                                distanceNormal.Add(idx);
                            }
                            else if(distanceLabels[idx] == DISTANCE_HOOK)
                            {
                                distanceHook.Add(idx);
                            }
                            else if (distanceLabels[idx] == DISTANCE_MODEL)
                            {
                                distanceModel.Add(idx);
                            }
                            else
                            {
                            }
                            idx++;
                        }

                        distanceInfo = new StDistance[sizeDistanceInfo / 30];
                        idx = 0;
                        for (int pos = posVInfo + posDistanceInfo; pos < posVInfo + posDistanceInfo + sizeDistanceInfo; pos += 30)
                        {
                            distanceInfo[idx] = new StDistance();
                            distanceInfo[idx].distance = BitConverter.ToSingle(buffer, pos);
                            distanceInfo[idx].ClusterIndex = (int)buffer[pos + 4];
                            distanceInfo[idx].CraneIndex = (int)buffer[pos + 5];
                            distanceInfo[idx].xCluster = BitConverter.ToSingle(buffer, pos + 6);
                            distanceInfo[idx].yCluster = BitConverter.ToSingle(buffer, pos + 10);
                            distanceInfo[idx].zCluster = BitConverter.ToSingle(buffer, pos + 14);
                            distanceInfo[idx].xCrane = BitConverter.ToSingle(buffer, pos + 18);
                            distanceInfo[idx].yCrane = BitConverter.ToSingle(buffer, pos + 22);
                            distanceInfo[idx].zCrane = BitConverter.ToSingle(buffer, pos + 26);
                            idx++;
                        }

                        collisionInfo = new byte[numCollision];
                        idx = 0;
                        for (int pos = posVInfo + posCollision; pos < posVInfo + posCollision + numCollision; pos++)
                        {
                            collisionInfo[idx] = buffer[pos];
                            idx++;
                        }

                        reasonInfo = new byte[numReasonInfo];
                        idx = 0;
                        for (int pos = posVInfo + posReasonInfo; pos < posVInfo + posReasonInfo + numReasonInfo; pos++)
                        {
                            reasonInfo[idx] = buffer[pos];
                            idx++;
                        }
                    }
                }
            }
        }
    }
}