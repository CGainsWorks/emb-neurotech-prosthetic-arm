//
// Copyright (c) 2026 CGainsWorks
//
// This file is licensed under the MIT License.
//
// The host side of the serial link, replayed into a UART in virtual time.
//
// The board will not drive a servo until a host has commanded it and it stops
// again when the commands stop, so the PWM path cannot be exercised without a
// host. This plays one: a file of timed byte strings, each written into the
// UART's receiver at its virtual time. One line per frame:
//
//     <time in microseconds> <bytes in hex, no spaces>
//
// It sits on the system bus only so the monitor can reach it by name (Renode
// does not name peripherals declared at "none"): a 16-byte window in the FMC
// bank 1 region, where nothing is fitted on this board, which reads back how
// many frames have been delivered. The firmware never touches it.
//
// The bytes are produced by firmware/tools/pa_link.cpp, which encodes them with
// the firmware's own protocol code, so what arrives here is exactly what the
// ROS 2 bridge would send down a cable.
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;

using Antmicro.Renode.Core;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Peripherals.UART;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class HostLinkReplay : IDoubleWordPeripheral, IKnownSize
    {
        public HostLinkReplay(IMachine machine, IUART uart)
        {
            this.uart = uart;
            timer = new LimitTimer(machine.ClockSource, 1000000, this, "replay", limit: 1,
                direction: Direction.Ascending, enabled: false, workMode: WorkMode.OneShot, eventEnabled: true);
            timer.LimitReached += Deliver;
        }

        public void Reset()
        {
            timer.Reset();
            next = 0;
            nowUs = 0;
            FramesDelivered = 0;
        }

        public uint ReadDoubleWord(long offset)
        {
            return offset == 0 ? (uint)FramesDelivered : 0;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
        }

        public long Size => 0x10;

        // Read the schedule. Call before the emulation starts: times are from
        // virtual time zero.
        public void Load(string path)
        {
            schedule.Clear();
            foreach(var raw in File.ReadAllLines(path))
            {
                var line = raw.Trim();
                if(line.Length == 0 || line.StartsWith("#"))
                {
                    continue;
                }
                var parts = line.Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
                if(parts.Length != 2 || parts[1].Length % 2 != 0)
                {
                    throw new RecoverableException($"Malformed line in {path}: {line}");
                }
                var time = ulong.Parse(parts[0], CultureInfo.InvariantCulture);
                var bytes = new byte[parts[1].Length / 2];
                for(var i = 0; i < bytes.Length; i++)
                {
                    bytes[i] = byte.Parse(parts[1].Substring(2 * i, 2), NumberStyles.HexNumber, CultureInfo.InvariantCulture);
                }
                schedule.Add(Tuple.Create(time, bytes));
            }
            schedule.Sort((a, b) => a.Item1.CompareTo(b.Item1));
            next = 0;
            nowUs = 0;
            this.Log(LogLevel.Info, "Loaded {0} frames for the host link", schedule.Count);
            Arm();
        }

        public ulong FramesDelivered { get; private set; }

        private void Arm()
        {
            if(next >= schedule.Count)
            {
                timer.Enabled = false;
                return;
            }
            var due = schedule[next].Item1;
            timer.Value = 0;
            timer.Limit = due > nowUs ? due - nowUs : 1;
            timer.Enabled = true;
        }

        private void Deliver()
        {
            nowUs = schedule[next].Item1 > nowUs ? schedule[next].Item1 : nowUs + 1;
            while(next < schedule.Count && schedule[next].Item1 <= nowUs)
            {
                foreach(var b in schedule[next].Item2)
                {
                    uart.WriteChar(b);
                }
                FramesDelivered++;
                next++;
            }
            Arm();
        }

        private int next;
        private ulong nowUs;
        private readonly List<Tuple<ulong, byte[]>> schedule = new List<Tuple<ulong, byte[]>>();
        private readonly LimitTimer timer;
        private readonly IUART uart;
    }
}
