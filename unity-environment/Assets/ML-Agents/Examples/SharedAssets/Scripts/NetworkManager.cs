using System.Collections.Generic;
using UnityEngine;
using System.Net.Sockets;
using System.Text;

namespace Assets.ML_Agents.Examples.SharedAssets.Scripts
{
    public class NetworkManager
    {
        const int messageLength = 12000;
        const bool showDebug = false;

        Socket sender = null;
        StringBuilder rMessageString = new StringBuilder(messageLength);
        byte[] messageHolder = new byte[messageLength];
        byte[] lengthHolder = new byte[4];

        bool isConnected = false;
        public bool IsConnected
        {
            get { return isConnected; }
            set { isConnected = value; }
        }

        public NetworkManager()
        {
            sender = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
        }

        public void Close()
        {
            sender.Close();
            //    sender.Shutdown(SocketShutdown.Both);
        }

        public void ConnectToLocalHost()
        {
            if (isConnected)
                return;
            try
            {
                sender.Connect("localhost", 5555);
                isConnected = true;
            }
            catch
            {
                if (showDebug)
                    UnityEngine.Debug.Log("Not Connected");
                isConnected = false;
            }
        }

        public string Send(ref string message)
        {
            try
            {
                SendBySocket(ref message);
                return RecieveFromSocket();
            }
            catch
            {
                if (isConnected)
                {
                    if (showDebug)
                        UnityEngine.Debug.Log("Not Connected");
                    isConnected = false;
                    sender.Disconnect(false);
                    sender.Close();
                    sender = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
                }
            }
            return "";
        }

        private byte[] AppendLength(byte[] input)
        {
            byte[] newArray = new byte[input.Length + 4];
            input.CopyTo(newArray, 4);
            System.BitConverter.GetBytes(input.Length).CopyTo(newArray, 0);
            return newArray;
        }

        void SendBySocket(ref string message)
        {
            if (sender != null && isConnected)
            {
                sender.Send(AppendLength(Encoding.ASCII.GetBytes(message)));
            }
        }

        string RecieveFromSocket()
        {
            sender.Receive(lengthHolder);
            int totalLength = System.BitConverter.ToInt32(lengthHolder, 0);
            int location = 0;
            rMessageString.Clear();

            int counter_try = 0;
            while (location != totalLength && counter_try < 10)
            {
                int fragment = sender.Receive(messageHolder);
                location += fragment;
                rMessageString.Append(Encoding.ASCII.GetString(messageHolder, 0, fragment));

                if (fragment == 0)
                    counter_try++;
                else
                    counter_try = 0;
            }
            return rMessageString.ToString();
        }
    };


}
