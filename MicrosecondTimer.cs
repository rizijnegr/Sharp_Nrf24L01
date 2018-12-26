using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;

namespace NRF24L01Plus
{
    public class MicrosecondTimer
    {
        private readonly static double _cyclesPerMIcrosecond;

        static MicrosecondTimer()
        {
            int cycles = 20000000;
            var timer = Stopwatch.StartNew();
            for (int i = 0; i < cycles; i++) { }
            timer.Stop();
            Console.WriteLine($"Stopwatch.Frequency: {Stopwatch.Frequency}");
            var microSecondsPerTick = (1000d * 1000d) / Stopwatch.Frequency;
            Console.WriteLine($"microSecondsPerTick: {microSecondsPerTick}");
            var elapsedMicroseconds = timer.ElapsedTicks * microSecondsPerTick;
            Console.WriteLine($"elapsedMicroseconds: {elapsedMicroseconds}");
            _cyclesPerMIcrosecond = cycles / elapsedMicroseconds;
            Console.WriteLine($"Cycles per microsecond: {_cyclesPerMIcrosecond}");
        }

        public static void Wait(long microseconds)
        {
            var cycles = (int)(_cyclesPerMIcrosecond * microseconds);
            for (int i = 0; i < cycles; i++) { };
        }
    }
}
