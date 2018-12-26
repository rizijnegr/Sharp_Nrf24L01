using System;
using System.Collections.Generic;
using System.Text;

namespace NRF24L01Plus
{
    public class FifoStatus
    {
        private const byte FIFO_STATUS_REGISTER = 0x17;

        public bool TXReuse { get; private set; }
        public bool TXFull  { get; private set; }
        public bool TXEmpty { get; private set; }
        public bool RXFull  { get; private set; }
        public bool RXEmpty { get; private set; }

        public FifoStatus(bool txReuse, bool txFull, bool txEmpty, bool rxFull, bool rxEmpty)
        {
            TXReuse = txReuse;
            TXFull  = txFull;
            TXEmpty = txEmpty;
            RXFull  = rxFull;
            RXEmpty = rxEmpty;
        }

        public static FifoStatus Parse(NRF24L01 radioModule)
        {
            var rawValue = radioModule.ReadRegister(FIFO_STATUS_REGISTER);
            return new FifoStatus(rawValue.BitIsTrue(7),
                                  rawValue.BitIsTrue(6),
                                  rawValue.BitIsTrue(5),
                                  rawValue.BitIsTrue(2),
                                  rawValue.BitIsTrue(1));
        }
    }
}
