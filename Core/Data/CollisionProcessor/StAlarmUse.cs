using System;
using System.Net.Sockets;
using System.Net;
namespace CsCore
{
    namespace Data
    {
        namespace CollisionProcessor
        {
            public class StAlarmUse
            {
                public int type { get; private set; } = 0x02;
                public int code { get; private set; } = 0x0d;
                public double timestamp;
                public byte[] ipAddress;
                public bool bAlarmUse;

                public StAlarmUse(double timestamp, bool bAlarmUse)
                {
                    string ip = GetLocalIPAddress();
                    if (ip != null)
                        ipAddress = IPAddress.Parse(GetLocalIPAddress()).GetAddressBytes();
                    else
                        ipAddress = new byte[4] { 0, 0, 0, 0 };

                    this.timestamp = timestamp;
                    this.bAlarmUse = bAlarmUse;
                }
                string GetLocalIPAddress()
                {
                    var host = Dns.GetHostEntry(Dns.GetHostName());
                    foreach (var ip in host.AddressList)
                    {
                        // IPv4 + 루프백 주소(127.0.0.1) 제외
                        if (ip.AddressFamily == AddressFamily.InterNetwork &&
                            !IPAddress.IsLoopback(ip))
                        {
                            return ip.ToString();
                        }
                    }
                    return null;
                }
                public int GetBytes(ref byte[] buffer, int offset, int length)
                {
                    int size = 0;
                    if (length >= 9)
                    {
                        //byte[] byteTimestamp = BitConverter.GetBytes(timestamp);
                        //Array.Copy(byteTimestamp, 0, buffer, offset, 8);

                        // IP (4바이트)
                        Buffer.BlockCopy(ipAddress, 0, buffer, offset, 4);
                        // Port (ushort 2바이트)
                        var portBytes = BitConverter.GetBytes(0);
                        Buffer.BlockCopy(portBytes, 0, buffer, offset + 4, 2);

                        // Reserved (ushort 2바이트)
                        var reservedBytes = BitConverter.GetBytes(0);
                        Buffer.BlockCopy(reservedBytes, 0, buffer, offset + 6, 2);

                        if (bAlarmUse)
                        {
                            buffer[offset + 8] = 1;
                        }
                        else
                        {
                            buffer[offset + 8] = 0;
                        }
                        size = 9;
                    }
                    return size;
                }
            }
        }
    }
}