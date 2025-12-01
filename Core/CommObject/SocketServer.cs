using System;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Collections.Generic;

namespace CsCore
{
    interface ISocketServer
    {
        void OnServerConnected(string ip, int port);
        void OnServerDisconnected(string ip, int port);
        void OnServerData(NetworkStream stream, byte[] buffer, int length);
    }

    class ClientObject
    {
        public TcpClient client;
        public NetworkStream stream;
        public ISocketServer observer;
        public byte[] readBuffer;
        public ClientObject(TcpClient client, int bufferSize, ISocketServer observer)
        {
            this.client = client;
            this.observer = observer;
            stream = client.GetStream();
            readBuffer = new byte[bufferSize];
        }
        public bool IsConnect()
        {
            return client.Connected;
        }
        public IAsyncResult BeginRead(AsyncCallback callback, object state)
        {
            return stream.BeginRead(readBuffer, 0, readBuffer.Length, callback, state);
        }
    }

    class SocketServer
    {
        private TcpListener listener;
        public List<ClientObject> clientObjects { get; } = new List<ClientObject>();
        public ISocketServer observer = null;
        public int packetSize { get; set; } = 4096;

        public SocketServer(ISocketServer observer)
        {
            this.observer = observer;
        }

        ~SocketServer()
        {
            Destroy();
        }
        public void Destroy()
        {
            listener?.Stop();
            listener?.Server?.Close();
            foreach (var clientObject in clientObjects)
            {
                clientObject?.client?.Close();
            }
        }

        public bool Create(string ip, int port)
        {
            bool ret = false;
            listener = new TcpListener(IPAddress.Parse(ip), port);
            try
            {
                listener.Start();
                listener.BeginAcceptTcpClient(new AsyncCallback(OnAcceptCallback), this);
                ret = true;
            }
            catch(Exception){}
            return ret;
        }

        public bool IsConnect()
        {
            bool result = true;

            foreach (ClientObject clientObject in clientObjects)
            {
                if (clientObject?.client?.Connected == false)
                    return false;
            }
            return result;
        }

        public void Write(byte[] buffer, int offset, int size)
        {
            foreach(var clientObject in clientObjects)
            {
                if(clientObject.client.Connected)
                {
                    clientObject.stream.Write(buffer, offset, size);
                }
            }
        }

        public static void OnAcceptCallback(IAsyncResult ar)
        {
            SocketServer server = (SocketServer)ar.AsyncState;
            try
            {
                ClientObject clientObject = new ClientObject(server.listener.EndAcceptTcpClient(ar), server.packetSize, server.observer);
                server.clientObjects.Add(clientObject);

                try
                {
                    if (server.observer != null)
                    {
                        IPEndPoint endpoint = (IPEndPoint)clientObject.client.Client.RemoteEndPoint;
                        server.observer.OnServerConnected(endpoint.Address.ToString(), endpoint.Port);
                    }
                }
                catch (Exception) { }

                try
                {
                    clientObject.BeginRead(new AsyncCallback(OnReadCallback), clientObject);
                }
                catch (Exception) { }

                try
                {
                    server.listener.BeginAcceptTcpClient(new AsyncCallback(OnAcceptCallback), server);
                }
                catch (Exception) { }

                foreach (var client in server.clientObjects)
                {
                    if (client.client.Connected == false)
                    {
                        server.clientObjects.Remove(client);
                    }
                }
            }
            catch (Exception) { }
        }

        public static void OnReadCallback(IAsyncResult ar)
        {
            try
            {
                ClientObject clientObject = (ClientObject)ar.AsyncState;
                int nRead = clientObject.stream.EndRead(ar);

                if (nRead > 0)
                {
                    try
                    {
                        if (clientObject.observer != null)
                        {
                            clientObject.observer.OnServerData(clientObject.stream, clientObject.readBuffer, nRead);
                        }
                    }
                    catch (Exception) { }

                    try
                    {
                        clientObject.BeginRead(new AsyncCallback(OnReadCallback), clientObject);
                    }
                    catch (Exception) { }
                }
                else
                {

                    try
                    {
                        if (clientObject.observer != null)
                        {
                            IPEndPoint endpoint = (IPEndPoint)clientObject.client.Client.RemoteEndPoint;
                            clientObject.observer.OnServerDisconnected(endpoint.Address.ToString(), endpoint.Port);
                        }
                    }
                    catch (Exception) { }
                    clientObject.client.Close();
                }
            }
            catch (Exception) { }
        }

    }
}
