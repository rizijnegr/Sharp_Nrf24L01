using System;
using System.Collections.Generic;
using System.Text;

namespace NRF24L01Plus
{
    /// <summary>
    /// Data Rate
    /// </summary>
    public enum DataRate
    {
        /// <summary>
        /// 1Mbps
        /// </summary>
        DR1Mbps = 0,
        /// <summary>
        /// 2Mbps
        /// </summary>
        DR2Mbps = 8,

        /// <summary>
        /// 250 Kbps
        /// </summary>
        DR250Kbps = 32
    }
}
