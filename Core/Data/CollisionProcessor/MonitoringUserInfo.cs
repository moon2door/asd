using System;

namespace CsCore
{
    namespace Data
    {
        namespace CollisionProcessor
        {
            public class MonitoringUserInfo
            {
                public string id = "";
                public int authority = 0;
                public string name = "";
                public string password = "";
                public string info = "";

                public MonitoringUserInfo()
                {
                }
                public int GetBytes(ref byte[] buffer, int offset, int length)
                {
                    int size = 0;
                    if (length >= 132)
                    {
                        byte[] byteId = System.Text.Encoding.UTF8.GetBytes(id);
                        byte[] byteName = System.Text.Encoding.UTF8.GetBytes(name);
                        byte[] byteAuthority = BitConverter.GetBytes(authority);
                        byte[] bytePassword = System.Text.Encoding.UTF8.GetBytes(password);
                        byte[] byteInfo = System.Text.Encoding.UTF8.GetBytes(info);

                        Array.Copy(byteId, 0, buffer, offset, Math.Min(byteId.Length, 32));
                        Array.Copy(byteAuthority, 0, buffer, offset + 32, Math.Min(byteAuthority.Length, 32));
                        Array.Copy(byteName, 0, buffer, offset + 36, Math.Min(byteName.Length, 32));
                        Array.Copy(bytePassword, 0, buffer, offset + 68, Math.Min(bytePassword.Length, 32));
                        Array.Copy(byteInfo, 0, buffer, offset + 100, Math.Min(byteInfo.Length, 32));
                        size = 132;
                    }
                    return size;
                }
            }
        }
    }
}