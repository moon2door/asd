using System;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Threading;

namespace CsCore
{
    interface ISocketClient
    {
        public abstract void OnConnected(string ip, int port);
        public abstract void OnDisconnected(string ip, int port);
        public abstract void OnServerData(byte[] data);
    }

    class SocketClient
    {
        static bool _isSocketClosed = false;
        private Socket socket;
        private IPEndPoint remoteEP;
        private Thread threadRead;
        private byte[] buffer;
        private ISocketClient observer;
        private int packetSize;
        private bool bRunThread = true;

        public SocketClient(string ip, int port, ISocketClient observer = null)
        {
            IPAddress ipAddress;
            if (IPAddress.TryParse(ip, out ipAddress) == false)
            {
                ipAddress = IPAddress.Parse("127.0.0.1");
            }
            remoteEP = new IPEndPoint(ipAddress, port);
            socket = new Socket(ipAddress.AddressFamily, SocketType.Stream, ProtocolType.Tcp);
            threadRead = new Thread(ThreadRead);
            threadRead.IsBackground = true;
            //threadRead.Start(this); //pjh
            packetSize = 4096;
            buffer = new byte[packetSize];
            this.observer = observer;
        }

        public void Destroy()
        {
            bRunThread = false;
            ////pjh
            //threadRead?.Abort();

            if (socket != null)
            {
                try
                {
                    if (socket.Connected)
                    {
                        socket.Shutdown(SocketShutdown.Both);
                    }
                }
                catch (SocketException ex)
                {
                    Console.WriteLine($"소켓 중지 중 예외 발생: {ex.Message}");
                }
                catch (ObjectDisposedException ex)
                {
                    Console.WriteLine($"소켓이 이미 해제됨: {ex.Message}");
                }
                finally
                {
                    socket.Close();
                    socket.Dispose();
                    _isSocketClosed = true;
                }
            }

            //~pjh
        }

        public void SetPacketSize(int packetSize)
        {
            if (packetSize > 0)
            {
                this.packetSize = packetSize;
                buffer = new byte[packetSize];
            }
            else
            {   
                this.packetSize = 4096;
                buffer = new byte[4096];
            }
        }

        public void Connect()
        {
            socket.BeginConnect(remoteEP, ConnectCallback, this);
        }

        private static void ConnectCallback(IAsyncResult ar)
        {
            SocketClient client = (SocketClient)ar.AsyncState;
            client.StopThread();
            try
            {
                client.socket.EndConnect(ar);
                if (client.observer != null)
                {
                    IPEndPoint ep = (IPEndPoint)client.remoteEP;
                    client.StartThread();//pjh
                    client.observer.OnConnected(ep.Address.ToString(), ep.Port);
                    _isSocketClosed = false;//pjh
                }
            }
            catch (SocketException e)
            {
                //pjh
                UnityEngine.Debug.LogError(e.Message);
                UnityEngine.Debug.LogError("ip : " + client.remoteEP.Address.ToString() + " port : " + client.remoteEP.Port.ToString());
                client.socket.BeginConnect(client.remoteEP, ConnectCallback, client);
                //~pjh
            }
        }

        public bool IsConnected()
        {
            return socket.Connected;
        }

        public int Write(byte[] message, int size)
        {
            int ret = -1;
            if (_isSocketClosed || !socket.Connected) return ret;//pjh

            try
            {
                ret = socket.Send(message, size, SocketFlags.None);
            }
            catch (Exception e)
            {
                    UnityEngine.Debug.LogError(e.ToString());//pjh
            }
            return ret;
        }

        public int Read(ref byte[] bytes, int size, int pos = 0)
        {
            int ret = -1;
            if (_isSocketClosed || !socket.Connected) return ret;//pjh
            try
            {
                if(socket != null)
                    ret = socket.Receive(bytes, pos, size, SocketFlags.None);
            }
            catch (Exception e)
            {
                UnityEngine.Debug.Log(e.ToString());
            }
            return ret;
        }

        //pjh
        public void StartThread()
        {
            if (threadRead == null || !threadRead.IsAlive)
            {
                bRunThread = true;
                threadRead = new Thread(ThreadRead);
                threadRead.Start(this);
            }
            else
            {
                UnityEngine.Debug.Log("Thread is already running.");
            }
        }

        public void StopThread()
        {
            if(threadRead != null && threadRead.IsAlive)
            {
                bRunThread = false;
                threadRead.Join();
            }
        }
        //~pjh

        public static void ThreadRead(Object obj)
        {
            SocketClient client = (SocketClient)obj;
            bool bConnected = false;
            while (client.bRunThread)
            {
                if (_isSocketClosed) break;//pjh
                try
                {
                    //UnityEngine.Debug.Log("Socket Connected : " + client.socket.Poll(1000, SelectMode.SelectRead) + ", available : "+ client.socket.Available);
                    if (client.socket.Poll(1000, SelectMode.SelectRead))
                    {
                        int nRead = client.Read(ref client.buffer, client.packetSize);
                        if (nRead > 0)
                        {
                            // Process receive data
                            if (client.observer != null)
                            {
                                byte[] buffer = new byte[nRead];
                                Array.Copy(client.buffer, buffer, nRead);
                                client.observer.OnServerData(buffer);
                            }
                            bConnected = true;
                        }
                        else
                        {
                            if(bConnected)
                            {
                                // Reconnect
                                if (client.IsConnected())
                                {
                                    client.socket.Disconnect(true);
                                }
                                client.socket.Close();
                                client.socket = new Socket(client.remoteEP.Address.AddressFamily, SocketType.Stream, ProtocolType.Tcp);

                                if (client.observer != null)
                                {
                                    IPEndPoint ep = (IPEndPoint)client.remoteEP;
                                    client.observer.OnDisconnected(ep.Address.ToString(), ep.Port);
                                }
                                client.socket.BeginConnect(client.remoteEP, ConnectCallback, client);
                                bConnected = false;
                            }
                        }
                    }
                    else
                    {
                        if (bConnected)
                        {
                            // Detect disconnection
                            //pjh int nRecv = client.Read(ref client.buffer, 0);
                            // Reconnect
                            if (client.IsConnected() == false)
                            {
                                client.socket.Close();
                                client.socket = new Socket(client.remoteEP.Address.AddressFamily, SocketType.Stream, ProtocolType.Tcp);

                                if (client.observer != null)
                                {
                                    IPEndPoint ep = (IPEndPoint)client.remoteEP;
                                    client.observer.OnDisconnected(ep.Address.ToString(), ep.Port);
                                }
                                client.socket.BeginConnect(client.remoteEP, ConnectCallback, client);
                                bConnected = false;
                            }
                        }
                    }
                }
                catch(Exception e)
                {
                    UnityEngine.Debug.Log("Error = "+ e.Message + " / "+ e.StackTrace);
                }

                Thread.Sleep(100);
            }
        }
    }
}
