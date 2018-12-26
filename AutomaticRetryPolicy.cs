using System;
using System.Collections.Generic;
using System.Text;

namespace NRF24L01Plus
{
    public class AutomaticRetryPolicy
    {
        private const byte ARD = 4;
        private const byte SETUP_RETR = 0x04;

        private byte _maxRetries;
        private byte _delayCoefficient;

        public byte RawValue => (byte)((DelayCoefficient << ARD) | MaxRetries);

        public int DelayInMicroseconds => (DelayCoefficient + 1) * 250;

        public byte DelayCoefficient
        {
            get => _delayCoefficient;
            set
            {
                if (value > 15)
                    throw new ArgumentOutOfRangeException(nameof(value), "Delay coefficient should not exceed 15");
                _delayCoefficient = value;
            }
        }
        public byte MaxRetries
        {
            get => _maxRetries;
            set
            {
                if (value > 15)
                    throw new ArgumentOutOfRangeException(nameof(value), "Maximum retries count should not exceed 15");
                _maxRetries = value;
            }
        }

        /// <summary>
        /// Default constructor
        /// </summary>
        /// <param name="maxRetries"></param>
        /// <param name="delayCoefficient"></param>
        public AutomaticRetryPolicy(byte maxRetries, byte delayCoefficient)
        {
            MaxRetries = maxRetries;
            DelayCoefficient = delayCoefficient;
        }

        /// <summary>
        /// Persist policy
        /// </summary>
        /// <param name="radioModule"></param>
        public void Persist(NRF24L01 radioModule)
        {
            radioModule.WriteRegister(SETUP_RETR, RawValue);
        }

        /// <summary>
        /// Read automatic retry policy of the given module
        /// </summary>
        /// <param name="radioModule"></param>
        /// <returns></returns>
        public static AutomaticRetryPolicy Read(NRF24L01 radioModule)
        {
            var rawValue = radioModule.ReadRegister(SETUP_RETR);
            byte retryCount = (byte)(rawValue & 0xf);
            byte delayCoefficient = (byte)(rawValue >> ARD);
            return new AutomaticRetryPolicy(retryCount, delayCoefficient);
        }
    }
}
