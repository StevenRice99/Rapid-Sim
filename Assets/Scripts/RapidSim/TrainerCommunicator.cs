using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

namespace RapidSim
{
    public class TrainerCommunicator : MonoBehaviour
    {
        private static TrainerCommunicator Singleton
        {
            get
            {
                if (_singleton != null)
                {
                    return _singleton;
                }

                GameObject go = new("Trainer Communicator");
                TrainerCommunicator c = go.AddComponent<TrainerCommunicator>();
                _singleton = c;
                return c;
            }
        }

        private static TrainerCommunicator _singleton;

        private Thread _mThread;
        public string connectionIP = "127.0.0.1";
        public int connectionPort = 1234;
        private TcpListener _listener;
        private TcpClient _client;

        private bool _running;

        private void Start()
        {
            ThreadStart ts = GetInfo;
            _mThread = new(ts);
            _mThread.Start();
        }

        private void GetInfo()
        {
            IPAddress.Parse(connectionIP);
            _listener = new(IPAddress.Any, connectionPort);
            _listener.Start();

            _client = _listener.AcceptTcpClient();

            _running = true;
            while (_running)
            {
                SendAndReceiveData();
            }
            _listener.Stop();
        }

        private void SendAndReceiveData()
        {
            NetworkStream nwStream = _client.GetStream();
            byte[] buffer = new byte[_client.ReceiveBufferSize];

            
            print("Reading...");
            int bytesRead = nwStream.Read(buffer, 0, _client.ReceiveBufferSize);
            string dataReceived = Encoding.UTF8.GetString(buffer, 0, bytesRead);

            int value = StringToNumber(dataReceived);
            print($"Received {value}");

            float[] values = {1f, 2f, 3f, 4f, 5f, 6f};
            string s = ArrayToString(values);
            byte[] myWriteBuffer = Encoding.ASCII.GetBytes(s);
            nwStream.Write(myWriteBuffer, 0, myWriteBuffer.Length);
        }

        private static int StringToNumber(string s)
        {
            if (s.StartsWith("(") && s.EndsWith(")"))
            {
                s = s.Substring(1, s.Length - 2);
            }

            return int.Parse(s);
        }

        private static string ArrayToString(float[] values)
        {
            string s = string.Empty;
            for (int i = 0; i < values.Length; i++)
            {
                s += $"{values[i]}";
                if (i < values.Length - 1)
                {
                    s += ",";
                }
            }

            return s;
        }
    }
}