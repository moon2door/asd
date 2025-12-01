using System;

namespace CsCore
{
    namespace Data
    {
        namespace CollisionProcessor
        {
            public struct RotorParameterSingle
            {
                public float intrinsic00;
                public float intrinsic01;
                public float intrinsic02;
                public float intrinsic03;
                public float intrinsic10;
                public float intrinsic11;
                public float intrinsic12;
                public float intrinsic13;
                public float intrinsic20;
                public float intrinsic21;
                public float intrinsic22;
                public float intrinsic23;
                public float intrinsic30;
                public float intrinsic31;
                public float intrinsic32;
                public float intrinsic33;

                public float extrinsic00;
                public float extrinsic01;
                public float extrinsic02;
                public float extrinsic03;
                public float extrinsic10;
                public float extrinsic11;
                public float extrinsic12;
                public float extrinsic13;
                public float extrinsic20;
                public float extrinsic21;
                public float extrinsic22;
                public float extrinsic23;
                public float extrinsic30;
                public float extrinsic31;
                public float extrinsic32;
                public float extrinsic33;

            }

            public class RotorParameter
            {
                public static int type = 0x02;
                public static int code = 0x11;

                public RotorParameterSingle[] param = new RotorParameterSingle[5];
                public RotorParameter()
                {
                }
                public RotorParameter(byte[] buffer)
                {
                    FromByte(buffer);
                }

                public int GetBytes(ref byte[] buffer, int offset, int length)
                {
                    var size = 0;
                    if (length < 4 * 32 * 5) return size;
                    
                    size = 4 * 32 * 5;

                    for (int i = 0; i < 5; i++)
                    {
                        /*var */byte[] byteintrinsic00 = BitConverter.GetBytes(param[i].intrinsic00);
                        /*var */byte[] byteintrinsic01 = BitConverter.GetBytes(param[i].intrinsic01);
                        /*var */byte[] byteintrinsic02 = BitConverter.GetBytes(param[i].intrinsic02);
                        /*var */byte[] byteintrinsic03 = BitConverter.GetBytes(param[i].intrinsic03);
                        /*var */byte[] byteintrinsic10 = BitConverter.GetBytes(param[i].intrinsic10);
                        /*var */byte[] byteintrinsic11 = BitConverter.GetBytes(param[i].intrinsic11);
                        /*var */byte[] byteintrinsic12 = BitConverter.GetBytes(param[i].intrinsic12);
                        /*var */byte[] byteintrinsic13 = BitConverter.GetBytes(param[i].intrinsic13);
                        /*var */byte[] byteintrinsic20 = BitConverter.GetBytes(param[i].intrinsic20);
                        /*var */byte[] byteintrinsic21 = BitConverter.GetBytes(param[i].intrinsic21);
                        /*var */byte[] byteintrinsic22 = BitConverter.GetBytes(param[i].intrinsic22);
                        /*var */byte[] byteintrinsic23 = BitConverter.GetBytes(param[i].intrinsic23);
                        /*var */byte[] byteintrinsic30 = BitConverter.GetBytes(param[i].intrinsic30);
                        /*var */byte[] byteintrinsic31 = BitConverter.GetBytes(param[i].intrinsic31);
                        /*var */byte[] byteintrinsic32 = BitConverter.GetBytes(param[i].intrinsic32);
                        /*var */byte[] byteintrinsic33 = BitConverter.GetBytes(param[i].intrinsic33);

                        /*var */byte[] byteextrinsic00 = BitConverter.GetBytes(param[i].extrinsic00);
                        /*var */byte[] byteextrinsic01 = BitConverter.GetBytes(param[i].extrinsic01);
                        /*var */byte[] byteextrinsic02 = BitConverter.GetBytes(param[i].extrinsic02);
                        /*var */byte[] byteextrinsic03 = BitConverter.GetBytes(param[i].extrinsic03);
                        /*var */byte[] byteextrinsic10 = BitConverter.GetBytes(param[i].extrinsic10);
                        /*var */byte[] byteextrinsic11 = BitConverter.GetBytes(param[i].extrinsic11);
                        /*var */byte[] byteextrinsic12 = BitConverter.GetBytes(param[i].extrinsic12);
                        /*var */byte[] byteextrinsic13 = BitConverter.GetBytes(param[i].extrinsic13);
                        /*var */byte[] byteextrinsic20 = BitConverter.GetBytes(param[i].extrinsic20);
                        /*var */byte[] byteextrinsic21 = BitConverter.GetBytes(param[i].extrinsic21);
                        /*var */byte[] byteextrinsic22 = BitConverter.GetBytes(param[i].extrinsic22);
                        /*var */byte[] byteextrinsic23 = BitConverter.GetBytes(param[i].extrinsic23);
                        /*var */byte[] byteextrinsic30 = BitConverter.GetBytes(param[i].extrinsic30);
                        /*var */byte[] byteextrinsic31 = BitConverter.GetBytes(param[i].extrinsic31);
                        /*var */byte[] byteextrinsic32 = BitConverter.GetBytes(param[i].extrinsic32);
                        /*var */byte[] byteextrinsic33 = BitConverter.GetBytes(param[i].extrinsic33);

                        Array.Copy(byteintrinsic00, 0/*0 + 128 * i*/, buffer, offset + 128 * i + 0/*i * 128 + 0*/, 4);
                        Array.Copy(byteintrinsic01, 0/*4 + 128 * i*/, buffer, offset + 128 * i + 4 /*i * 128 + 4*/, 4);
                        Array.Copy(byteintrinsic02, 0/*8 + 128 * i*/, buffer, offset + 128 * i + 8/*i * 128 + 8*/, 4);
                        Array.Copy(byteintrinsic03, 0/*12 + 128 * i*/, buffer, offset + 128 * i + 12/*i * 128 + 12*/, 4);
                        Array.Copy(byteintrinsic10, 0/*16 + 128 * i*/, buffer, offset + 128 * i + 16/*i * 128 + 16*/, 4);
                        Array.Copy(byteintrinsic11, 0/*20 + 128 * i*/, buffer, offset + 128 * i + 20/*i * 128 + 20*/, 4);
                        Array.Copy(byteintrinsic12, 0/*24 + 128 * i*/, buffer, offset + 128 * i + 24/*i * 128 + 24*/, 4);
                        Array.Copy(byteintrinsic13, 0/*28 + 128 * i*/, buffer, offset + 128 * i + 28/*i * 128 + 28*/, 4);
                        Array.Copy(byteintrinsic20, 0/*32 + 128 * i*/, buffer, offset + 128 * i + 32/*i * 128 + 32*/, 4);
                        Array.Copy(byteintrinsic21, 0/*36 + 128 * i*/, buffer, offset + 128 * i + 36/*i * 128 + 36*/, 4);
                        Array.Copy(byteintrinsic22, 0/*40 + 128 * i*/, buffer, offset + 128 * i + 40/*i * 128 + 40*/, 4);
                        Array.Copy(byteintrinsic23, 0/*44 + 128 * i*/, buffer, offset + 128 * i + 44/*i * 128 + 44*/, 4);
                        Array.Copy(byteintrinsic30, 0/*48 + 128 * i*/, buffer, offset + 128 * i + 48/*i * 128 + 48*/, 4);
                        Array.Copy(byteintrinsic31, 0/*52 + 128 * i*/, buffer, offset + 128 * i + 52/*i * 128 + 52*/, 4);
                        Array.Copy(byteintrinsic32, 0/*56 + 128 * i*/, buffer, offset + 128 * i + 56/*i * 128 + 56*/, 4);
                        Array.Copy(byteintrinsic33, 0/*60 + 128 * i*/, buffer, offset + 128 * i + 60/*i * 128 + 60*/, 4);

                        Array.Copy(byteextrinsic00, 0/*64 + 128 * i*/, buffer, offset + 128 * i + 64/*i * 128 + 64*/, 4);
                        Array.Copy(byteextrinsic01, 0/*68 + 128 * i*/, buffer, offset + 128 * i + 68/*i * 128 + 68*/, 4);
                        Array.Copy(byteextrinsic02, 0/*72 + 128 * i*/, buffer, offset + 128 * i + 72/*i * 128 + 72*/, 4);
                        Array.Copy(byteextrinsic03, 0/*76 + 128 * i*/, buffer, offset + 128 * i + 76/*i * 128 + 76*/, 4);
                        Array.Copy(byteextrinsic10, 0/*40 + 128 * i*/, buffer, offset + 128 * i + 80/*i * 128 + 80*/, 4);
                        Array.Copy(byteextrinsic11, 0/*44 + 128 * i*/, buffer, offset + 128 * i + 84/*i * 128 + 84*/, 4);
                        Array.Copy(byteextrinsic12, 0/*48 + 128 * i*/, buffer, offset + 128 * i + 88/*i * 128 + 88*/, 4);
                        Array.Copy(byteextrinsic13, 0/*92 + 128 * i*/, buffer, offset + 128 * i + 92/*i * 128 + 92*/, 4);
                        Array.Copy(byteextrinsic20, 0/*96 + 128 * i*/, buffer, offset + 128 * i + 96/*i * 128 + 96*/, 4);
                        Array.Copy(byteextrinsic21, 0/*100 + 128 * i*/, buffer, offset + 128 * i + 100/*i * 128 + 100*/, 4);
                        Array.Copy(byteextrinsic22, 0/*104 + 128 * i*/, buffer, offset + 128 * i + 104/*i * 128 + 104*/, 4);
                        Array.Copy(byteextrinsic23, 0/*108 + 128 * i*/, buffer, offset + 128 * i + 108/*i * 128 + 108*/, 4);
                        Array.Copy(byteextrinsic30, 0/*112 + 128 * i*/, buffer, offset + 128 * i + 112/*i * 128 + 112*/, 4);
                        Array.Copy(byteextrinsic31, 0/*116 + 128 * i*/, buffer, offset + 128 * i + 116/*i * 128 + 116*/, 4);
                        Array.Copy(byteextrinsic32, 0/*120 + 128 * i*/, buffer, offset + 128 * i + 120/*i * 128 + 120*/, 4);
                        Array.Copy(byteextrinsic33, 0/*124 + 128 * i*/, buffer, offset + 128 * i + 124/*i * 128 + 124*/, 4);

                    }
                    return size;
                }


                public void FromByte(byte[] buffer)
                {
                    if (buffer.Length >= 32 * 4 * 5)
                    {
                        for(int i=0; i<5; i++)
                        {
                            param[i].intrinsic00 = BitConverter.ToSingle(buffer, i * 128 + 0 );
                            param[i].intrinsic01 = BitConverter.ToSingle(buffer, i * 128 + 4 );
                            param[i].intrinsic02 = BitConverter.ToSingle(buffer, i * 128 + 8 );
                            param[i].intrinsic03 = BitConverter.ToSingle(buffer, i * 128 + 12);
                            param[i].intrinsic10 = BitConverter.ToSingle(buffer, i * 128 + 16);
                            param[i].intrinsic11 = BitConverter.ToSingle(buffer, i * 128 + 20);
                            param[i].intrinsic12 = BitConverter.ToSingle(buffer, i * 128 + 24);
                            param[i].intrinsic13 = BitConverter.ToSingle(buffer, i * 128 + 28);
                            param[i].intrinsic20 = BitConverter.ToSingle(buffer, i * 128 + 32);
                            param[i].intrinsic21 = BitConverter.ToSingle(buffer, i * 128 + 36);
                            param[i].intrinsic22 = BitConverter.ToSingle(buffer, i * 128 + 40);
                            param[i].intrinsic23 = BitConverter.ToSingle(buffer, i * 128 + 44);
                            param[i].intrinsic30 = BitConverter.ToSingle(buffer, i * 128 + 48);
                            param[i].intrinsic31 = BitConverter.ToSingle(buffer, i * 128 + 52);
                            param[i].intrinsic32 = BitConverter.ToSingle(buffer, i * 128 + 56);
                            param[i].intrinsic33 = BitConverter.ToSingle(buffer, i * 128 + 60);
                                                                                     
                            param[i].extrinsic00 = BitConverter.ToSingle(buffer, i * 128 + 64 );
                            param[i].extrinsic01 = BitConverter.ToSingle(buffer, i * 128 + 68 );
                            param[i].extrinsic02 = BitConverter.ToSingle(buffer, i * 128 + 72 );
                            param[i].extrinsic03 = BitConverter.ToSingle(buffer, i * 128 + 76 );
                            param[i].extrinsic10 = BitConverter.ToSingle(buffer, i * 128 + 80 );
                            param[i].extrinsic11 = BitConverter.ToSingle(buffer, i * 128 + 84 );
                            param[i].extrinsic12 = BitConverter.ToSingle(buffer, i * 128 + 88 );
                            param[i].extrinsic13 = BitConverter.ToSingle(buffer, i * 128 + 92 );
                            param[i].extrinsic20 = BitConverter.ToSingle(buffer, i * 128 + 96 );
                            param[i].extrinsic21 = BitConverter.ToSingle(buffer, i * 128 + 100);
                            param[i].extrinsic22 = BitConverter.ToSingle(buffer, i * 128 + 104);
                            param[i].extrinsic23 = BitConverter.ToSingle(buffer, i * 128 + 108);
                            param[i].extrinsic30 = BitConverter.ToSingle(buffer, i * 128 + 112);
                            param[i].extrinsic31 = BitConverter.ToSingle(buffer, i * 128 + 116);
                            param[i].extrinsic32 = BitConverter.ToSingle(buffer, i * 128 + 120);
                            param[i].extrinsic33 = BitConverter.ToSingle(buffer, i * 128 + 124);
                        }
                    }
                }
            }
        }
    }
}