using System;
using System.Collections.Generic;
using System.Text;

namespace NRF24L01Plus
{
    public class ReceivedDataEventArgs : EventArgs
    {
        public byte[] Data { get; }
        public ReceivedDataEventArgs(byte[] data)
        {
            Data = data;
        }
    }
}
