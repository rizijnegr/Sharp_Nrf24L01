using System;
using System.Collections.Generic;
using System.Text;

namespace NRF24L01Plus
{
    static class MemoryExtensions
    {
        public static bool BitIsTrue(this byte value, int bitNumber)
        {
            return (value & (1 << bitNumber - 1)) != 0;
        }
    }
}
