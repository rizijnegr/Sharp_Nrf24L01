using System;
using System.Collections.Generic;
using System.Text;

namespace NRF24L01Plus
{
    public class Nrf24L01Config
    {
        private const byte CONFIG_REGISTER = 0x00;

        /// <summary>
        /// Raw register value
        /// </summary>
        public byte RawValue { get; private set; }

        /// <summary>
        /// MASK_RX_DR
        /// </summary>
        public bool MaskDataReady { get; set; } = false;

        /// <summary>
        /// MASK_TX_DS
        /// </summary>
        public bool MaskDataSent { get; set; } = false;

        /// <summary>
        /// MASK_MAX_RT
        /// </summary>
        public bool MaskMaximumRetries { get; set; } = false;

        /// <summary>
        /// EN_CRC
        /// </summary>
        public bool EnableCRC { get; set; } = true;

        /// <summary>
        /// CRC
        /// </summary>
        public CRCLength CRCLength { get; set; } = CRCLength.CRC8;

        /// <summary>
        /// PWR_UP
        /// </summary>
        public PowerMode Power { get; set; } = PowerMode.PowerDown;

        /// <summary>
        /// PRIM_RX
        /// </summary>
        public ChipWorkMode ChipMode { get; set; } = ChipWorkMode.Receive;

        public void Persist(NRF24L01 radioModule)
        {
            byte config = 0;
            if (MaskDataReady) config |= (1 << 6);
            if (MaskDataSent) config |= (1 << 5);
            if (MaskMaximumRetries) config |= (1 << 4);
            if (EnableCRC) config |= (1 << 3);
            if (CRCLength == CRCLength.CRC16) config |= (1 << 2);
            if (Power == PowerMode.PowerUp) config |= (1 << 1);
            if (ChipMode == ChipWorkMode.Receive) config |= 1;

            radioModule.WriteRegister(CONFIG_REGISTER, config);
        }

        /// <summary>
        /// Read configuration of the given radio module
        /// </summary>
        /// <param name="radioModule"></param>
        /// <returns></returns>
        public static Nrf24L01Config Read(NRF24L01 radioModule)
        {
            var cfg = radioModule.ReadRegister(CONFIG_REGISTER);
            Nrf24L01Config result = new Nrf24L01Config()
            {
                RawValue = cfg,
                MaskDataReady = cfg.BitIsTrue(7),
                MaskDataSent = cfg.BitIsTrue(6),
                MaskMaximumRetries = cfg.BitIsTrue(5),
                EnableCRC = cfg.BitIsTrue(4),
                CRCLength = cfg.BitIsTrue(3) ? CRCLength.CRC16 : CRCLength.CRC8,
                Power = cfg.BitIsTrue(2) ? PowerMode.PowerUp : PowerMode.PowerDown,
                ChipMode = cfg.BitIsTrue(1) ? ChipWorkMode.Receive : ChipWorkMode.Transfer
            };
            return result;
        }

        public override string ToString()
        {
            return "Config values:" + Environment.NewLine +
                $"  RawValue\t\t\t\t: 0x{RawValue:X2}" + Environment.NewLine +
                $"  MaskDataReady\t\t(MASK_RX_DR)\t: {MaskDataReady}" +  Environment.NewLine +
                $"  MaskDataSent\t\t(MASK_TX_DS)\t: {MaskDataSent}" + Environment.NewLine +
                $"  MaskMaximumRetries\t(MASK_MAX_RT)\t: {MaskMaximumRetries}" + Environment.NewLine +
                $"  EnableCRC\t\t(EN_CRC)\t: {EnableCRC}" + Environment.NewLine +
                $"  CRCLength\t\t(CRC0)\t\t: {CRCLength}" + Environment.NewLine +
                $"  Power\t\t\t(PWR_UP)\t: {Power}" + Environment.NewLine +
                $"  ChipMode\t\t(PRIM_RX)\t: {ChipMode}";
        }
    }

}
