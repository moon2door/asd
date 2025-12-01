using System;
using CsCore.IO;

namespace CsCore
{
    public interface ICollisionProcessor
    {
        public void OnConnected(string ip, int port);
        public void OnDisconnected(string ip, int port);
        public void OnSystemStatus(Data.CollisionProcessor.StSystemStatus status);
        public void OnCollisionProcessorDistance(Data.CollisionProcessor.StDistanceSocket compressed);

        public void OnCollisionProcessorDistanceCompressed(Data.CollisionProcessor.StDistanceSocketCompressed compressed);
        public void OnRotorParameter(Data.CollisionProcessor.RotorParameter param);
        public void OnCollisionZoneLength(Data.CollisionProcessor.CollisionZoneLength length);
        public void OnMaintenanceInfo(Data.CollisionProcessor.MaintenanceInfo info);
//cooperationMode
// plc
//collisionHistory
//OperationgHistory
//MonitoringUserInfo
//HeartBeat
//CarneAttitude
//MontioringCraneAlive
        public void OnCraneMiniInfo(Data.CollisionProcessor.StCraneMiniInfo info);
//ModelDistanceSocket
        public void OnMaxDistance(Data.CollisionProcessor.StMaxDistance info);
        public void OnMonitoringLabelInfo(Data.CollisionProcessor.StMonitoringLabelInfo info);

        public void OnRotorControlSocket(Data.CollisionProcessor.RotorControlSocket info);

    }
    public class InterfaceCollisionProcessor : ISocketClient
    {
        private byte[] payload = new byte[6553600];
        private SocketClient client;
        private ICollisionProcessor observer;

        public InterfaceCollisionProcessor(ICollisionProcessor observer = null)
        {
            this.observer = observer;
        }

        public void Create(string ip, int port)
        {
            client = new SocketClient(ip, port, this);
            //client.Create(ip, port, this);
            client.SetPacketSize(6);
            client.Connect();
        }

        public void Destroy()
        {
            client?.Destroy();//pjh
            client=null;//pjh
        }

        public bool IsConnect
        {
            get
            {
                if (client != null && client.IsConnected() == true) return true;
                return false;
            }
        }

        public void SendAlarmUse(CsCore.Data.CollisionProcessor.StAlarmUse data)
        {
            if (client == null) return;//pjh
            byte[] buffer = new byte[1024];
            int payloadSize = data.GetBytes(ref buffer, 6, 1024);

            CsCore.Data.StSocketHeader header = new CsCore.Data.StSocketHeader((byte)data.type, (byte)data.code, payloadSize);
            int headerSize = header.GetByte(ref buffer, 0, 1024);

            int size = payloadSize + headerSize;
            client.Write(buffer, size);
        }

        public void SendCollisionZoneLength(CsCore.Data.CollisionProcessor.CollisionZoneLength data)
        {
            if (client == null) return;//pjh
            byte[] buffer = new byte[1024];
            int payloadSize = data.GetBytes(ref buffer, 6, 1024);

            CsCore.Data.StSocketHeader header = new CsCore.Data.StSocketHeader((byte)0x02, (byte)0x06, payloadSize);
            int headerSize = header.GetByte(ref buffer, 0, 1024);

            int size = payloadSize + headerSize;
            client.Write(buffer, size);
        }

        public void SendRotorControl(CsCore.Data.CollisionProcessor.RotorControl data)
        {
            if (client == null) return;//pjh
            byte[] buffer = new byte[1024];
            int payloadSize = data.GetBytes(ref buffer, 6, 1024);

            CsCore.Data.StSocketHeader header = new CsCore.Data.StSocketHeader((byte)0x02, (byte)0x01, payloadSize); 
            int headerSize = header.GetByte(ref buffer, 0, 1024);

            int size = payloadSize + headerSize;
            client.Write(buffer, size);
        }

        public void SendRotorControlSocket(CsCore.Data.CollisionProcessor.RotorControlSocket data)
        {
            if (client == null) return;//pjh
            byte[] buffer = new byte[1024];
            int payloadSize = data.GetBytes(ref buffer, 6, 1024);

            CsCore.Data.StSocketHeader header = new CsCore.Data.StSocketHeader((byte)0x02, (byte)0x32, payloadSize);
            int headerSize = header.GetByte(ref buffer, 0, 1024);

            int size = payloadSize + headerSize;
            client.Write(buffer, size);
        }


        public void SendRequestRotorParameter()
        {
            if (client == null) return;//pjh
            byte[] buffer = new byte[1024];

            CsCore.Data.StSocketHeader header = new CsCore.Data.StSocketHeader((byte)0x02, (byte)0x10, 0);
            int headerSize = header.GetByte(ref buffer, 0, 1024);
            client.Write(buffer, headerSize);
        }

        public void SendRequestCollisionZoneLength(CsCore.Data.CollisionProcessor.CollisionRequestZone data)
        {
            if (client == null) return;//pjh
            byte[] buffer = new byte[1024];
            int payloadSize = data.GetBytes(ref buffer, 6, 1024);

            CsCore.Data.StSocketHeader header = new CsCore.Data.StSocketHeader((byte)0x02, (byte)0x12, payloadSize);
            int headerSize = header.GetByte(ref buffer, 0, 1024);

            int size = payloadSize + headerSize;
            client.Write(buffer, size);
        }

        public void SendReportMonitoringUserInfo(CsCore.Data.CollisionProcessor.MonitoringUserInfo data)
        {
            if (client == null) return;//pjh
            byte[] buffer = new byte[1024];
            int payloadSize = data.GetBytes(ref buffer, 6, 1024);

            CsCore.Data.StSocketHeader header = new CsCore.Data.StSocketHeader((byte)0x02, (byte)0x16, payloadSize);
            int headerSize = header.GetByte(ref buffer, 0, 1024);

            int size = payloadSize + headerSize;
            client.Write(buffer, size);
        }

        public void SendMonitoringLabelInfo(CsCore.Data.CollisionProcessor.StMonitoringLabelInfo data)
        {
            if (client == null) return;//pjh
            byte[] buffer = new byte[1024];
            int payloadSize = data.GetBytes(ref buffer, 6, 1024);

            CsCore.Data.StSocketHeader header = new CsCore.Data.StSocketHeader((byte)0x02, (byte)0x30, payloadSize);
            int headerSize = header.GetByte(ref buffer, 0, 1024);

            int size = payloadSize + headerSize;
            client.Write(buffer, size);
        }
        public void SendMaxDistance(CsCore.Data.CollisionProcessor.StMaxDistance data)
        {
            if (client == null) return;//pjh
            byte[] buffer = new byte[1024];
            int payloadSize = data.GetBytes(ref buffer, 6, 1024);

            CsCore.Data.StSocketHeader header = new CsCore.Data.StSocketHeader((byte)0x02, (byte)0x31, payloadSize);
            int headerSize = header.GetByte(ref buffer, 0, 1024);

            int size = payloadSize + headerSize;
            client.Write(buffer, size);
        }
        public void SendRequestSetNewParameter(CsCore.Data.CollisionProcessor.RotorParameter data)
        {
            if (client == null) return;//pjh
            byte[] buffer = new byte[1024];
            int payloadSize = data.GetBytes(ref buffer, 6, 1024);

            CsCore.Data.StSocketHeader header = new CsCore.Data.StSocketHeader((byte)0x02, (byte)0x50, payloadSize);
            int headerSize = header.GetByte(ref buffer, 0, 1024);

            int size = payloadSize + headerSize;
            client.Write(buffer, size);
        }

        void ISocketClient.OnConnected(string ip, int port)
        {
            observer.OnConnected(ip, port);
        }
        void ISocketClient.OnDisconnected(string ip, int port)
        {
            observer.OnDisconnected(ip, port);
        }
        void ISocketClient.OnServerData(byte[] data)
        {
            Data.StSocketHeader header = new Data.StSocketHeader(data);
            bool error = false;
            if (header.PayloadLength > 0)
            {
                int remained = header.PayloadLength;
                int offset = 0;
                while (remained > 0)
                {
                    int nRead = client.Read(ref payload, remained, offset);
                    if (nRead > 0)
                    {
                        remained -= nRead;
                        offset += nRead;
                    }
                    else
                    {
                        error = true;
                        break;
                    }
                }
            }

            if (error == false)
            {
                ParseHeader(header.Type, header.Code, payload, header.PayloadLength);
            }
        }

        void ParseHeader(int type, int code, byte[] payload, int length)
        {
            //UnityEngine.Debug.Log(String.Format("On message {0}:{1} -- {2}", type, code, length));
            if (type == Data.CollisionProcessor.StSystemStatus.type &&
                code == Data.CollisionProcessor.StSystemStatus.code)    
            {
                Data.CollisionProcessor.StSystemStatus status = new Data.CollisionProcessor.StSystemStatus(payload);
                if (observer != null)
                {
                    observer.OnSystemStatus(status);
                }
            }
            else if (type == Data.CollisionProcessor.StDistanceSocketCompressed.type &&
                code == Data.CollisionProcessor.StDistanceSocketCompressed.code)
            {
                Data.CollisionProcessor.StDistanceSocketCompressed data = new Data.CollisionProcessor.StDistanceSocketCompressed(payload);
                if (observer != null)
                {
                    
                    observer.OnCollisionProcessorDistanceCompressed(data);
                    //UnityEngine.Debug.Log("OnCollisionProcessorDistanceCompressed");
                }
            }
            else if (type == Data.CollisionProcessor.StCraneMiniInfo.type &&
                code == Data.CollisionProcessor.StCraneMiniInfo.code)
            {
                Data.CollisionProcessor.StCraneMiniInfo info = new Data.CollisionProcessor.StCraneMiniInfo(payload);
                if (observer != null)
                {
                    observer.OnCraneMiniInfo(info);
                    //UnityEngine.Debug.Log("OnCraneMiniInfo");
                }
            }
            else if (type == Data.CollisionProcessor.StDistanceSocket.type &&
                code == Data.CollisionProcessor.StDistanceSocket.code)
            {
                Data.CollisionProcessor.StDistanceSocket data = new Data.CollisionProcessor.StDistanceSocket(payload);
                if (observer != null)
                {
                    observer.OnCollisionProcessorDistance(data);
                    //UnityEngine.Debug.Log("OnCollisionProcessorDistance");
                }
                IO.DistanceLog.Write(payload, length);
            }
            else if(type == Data.CollisionProcessor.RotorParameter.type &&
                code == Data.CollisionProcessor.RotorParameter.code)
            {
                Data.CollisionProcessor.RotorParameter data = new Data.CollisionProcessor.RotorParameter(payload);
                if (observer != null)
                {
                    observer.OnRotorParameter(data);
                    //UnityEngine.Debug.Log("OnRotorParameter");
                }
            }
            else if (type == Data.CollisionProcessor.MaintenanceInfo.type &&
                code == Data.CollisionProcessor.MaintenanceInfo.code)
            {
                Data.CollisionProcessor.MaintenanceInfo data = new Data.CollisionProcessor.MaintenanceInfo(payload);
                if (observer != null)
                {
                    observer.OnMaintenanceInfo(data);
                    //UnityEngine.Debug.Log("OnMaintenanceInfo");
                }
            }
            else if (type == Data.CollisionProcessor.CollisionZoneLength.type &&
                code == Data.CollisionProcessor.CollisionZoneLength.code)
            {
                Data.CollisionProcessor.CollisionZoneLength data = new Data.CollisionProcessor.CollisionZoneLength(payload);
                if (observer != null)
                {
                    observer.OnCollisionZoneLength(data);
                    //UnityEngine.Debug.Log("OnCollisionZoneLength");
                }
            }
            else if (type == Data.CollisionProcessor.StMonitoringLabelInfo.type &&
                code == Data.CollisionProcessor.StMonitoringLabelInfo.code)
            {
                Data.CollisionProcessor.StMonitoringLabelInfo data = new Data.CollisionProcessor.StMonitoringLabelInfo(payload);
                if (observer != null)
                {
                    observer.OnMonitoringLabelInfo(data);
                    //UnityEngine.Debug.Log("OnMonitoringLabelInfo");
                }
            }
            else if (type == Data.CollisionProcessor.StMaxDistance.type &&
                     code == Data.CollisionProcessor.StMaxDistance.code)
            {
                Data.CollisionProcessor.StMaxDistance data = new Data.CollisionProcessor.StMaxDistance(payload);
                if (observer != null)
                {
                    observer.OnMaxDistance(data);
                }
            }
            else if (type == Data.CollisionProcessor.RotorControlSocket.type &&
                     code == Data.CollisionProcessor.RotorControlSocket.code)
            {
                Data.CollisionProcessor.RotorControlSocket data = new Data.CollisionProcessor.RotorControlSocket(payload);
                if (observer != null)
                {
                    observer.OnRotorControlSocket(data);
                }
            }
            //pjh
            // else if (type == 1 && code == 0x2B)
            // {
            //     //UnityEngine.Debug.Log("OnPlcControl");
            // }
            // else if (type == 4 && code == 22)
            // {
            //     //UnityEngine.Debug.Log("OnPlcControl");
            // }
            // else
            // {
            //     UnityEngine.Debug.Log(String.Format("On undefined message {0}:{1} -- {2}", type, code, length));
            // }
            //~Pjh
        }
    }
}
