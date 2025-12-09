//pjh
using System;
using System.Runtime.InteropServices;

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct StDistanceSocket
{
    public const int messageId = 19;
    public StCraneAttitude attitude;

    public StDistanceSocket(byte[] buffer)
    {
        attitude = new StCraneAttitude();
        attitude.FromByte(buffer);
    }
}
//~pjh