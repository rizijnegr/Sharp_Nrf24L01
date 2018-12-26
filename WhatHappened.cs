using System;
using System.Collections.Generic;
using System.Text;

namespace NRF24L01Plus
{
    public class WhatHappened
    {
        public bool NewDataReceived { get; set; }
        public bool DataTransfered { get; set; }
        public bool MaxRetriesExceeded { get; set; }

        public override string ToString()
        {
            return $" DataTransfered\t(TX_OK)\t: {DataTransfered}" +
                   $" MaxRetriesExceeded\t(MAX_RT)\t: {MaxRetriesExceeded}" +
                   $" NewDataReceived\t(RX_READY)\t: {NewDataReceived}";
        }
    }
}
