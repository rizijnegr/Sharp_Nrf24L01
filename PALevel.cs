using System;
using System.Collections.Generic;
using System.Text;

namespace NRF24L01Plus
{

    /// <summary>
    /// Power amplification level
    /// </summary>
    public enum PALevel
    {
        /// <summary>
        /// Minimal
        /// </summary>
        PA_MIN = 0,

        /// <summary>
        /// Low
        /// </summary>
        PA_LOW = 2,

        /// <summary>
        /// High
        /// </summary>
        PA_HIGH = 4,

        /// <summary>
        /// Maxmial
        /// </summary>
        PA_MAX = 6,

        /// <summary>
        /// Error
        /// </summary>
        PA_ERROR = 255
    }

}
