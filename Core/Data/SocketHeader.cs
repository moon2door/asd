using System;

namespace CsCore
{
    namespace Data
    {
        public class StSocketHeader
        {
            public StSocketHeader(byte[] buffer)
            {
                FromByte(buffer);
            }

            public StSocketHeader(byte type, byte code, int length)
            {
                Type = type;
                Code = code;
                PayloadLength = length;
            }

            public byte Type = 0;
            public byte Code = 0;
            public int PayloadLength = 0;
            public void FromByte(byte[] buffer)
            {
                if (buffer.Length >= 6)
                {
                    Type = buffer[0];
                    Code = buffer[1];
                    PayloadLength = BitConverter.ToInt32(buffer, 2);
                }
            }

            public int GetByte(ref byte[] buffer, int offset, int length)
            {
                int size = 0;
                if (length >= 6)
                {
                    byte[] bytePayloadLength = BitConverter.GetBytes(PayloadLength);
                    buffer[offset+0] = Type;
                    buffer[offset+1] = Code;
                    Array.Copy(bytePayloadLength, 0, buffer, offset+2, 4);
                    size = 6;
                }
                return size;
            }
        }
    }
}
