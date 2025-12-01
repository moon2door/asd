using System;

namespace CsCore
{
    public interface IIntegratedServer
    {
        public void OnConnected(string ip, int port);
        public void OnDisconnected(string ip, int port);
    }
    public class InterfaceIntegratedServer : ISocketServer
    {
        private byte[] payload = new byte[6553600];
        private SocketServer server;
        private IIntegratedServer observer;

        public InterfaceIntegratedServer(IIntegratedServer observer = null)
        {
            this.observer = observer;
        }

        public void Create(string ip, int port)
        {
            server = new SocketServer(this);
            server.Create(ip, port);
        }

        public void Destroy()
        {
            server?.Destroy();//pjh
            server=null;//pjh
        }

        public bool IsConnect
        {
            get
            {
                if (server != null && server.IsConnect() == true) return true;
                return false;
            }
        }

        public void SendDistanceBuffer(byte[] data, int length)
        {
            if (server == null) return;//pjh
            CsCore.Data.StSocketHeader header = new CsCore.Data.StSocketHeader((byte)CsCore.Data.CollisionProcessor.StDistanceSocket.type, (byte)CsCore.Data.CollisionProcessor.StDistanceSocket.code, length);
            int headerSize = header.GetByte(ref payload, 0, 6553600);
            Array.Copy(data, 0, payload, headerSize, length);
            int size = length + headerSize;
            server.Write(payload, 0, size);
        }

        void ISocketServer.OnServerConnected(string ip, int port)
        {
            observer?.OnConnected(ip, port);//pjh
        }

        void ISocketServer.OnServerDisconnected(string ip, int port)
        {
            observer?.OnDisconnected(ip, port);//pjh
        }

        void ISocketServer.OnServerData(System.Net.Sockets.NetworkStream stream, byte[] buffer, int length)
        {
        }
    }
}
