//
// Copyright (c) 2026 CGainsWorks
//
// This file is licensed under the MIT License.
//
// The voltage at the ADC pins of the eight sEMG channels, replayed from a
// stimulus file in virtual time.
//
// SemgStimulus holds the file. It sits on the system bus only so the monitor
// can reach it by name (Renode does not name peripherals declared at "none"):
// a 16-byte window in the FMC bank 1 region, where nothing is fitted on this
// board, which reads back how many samples have been taken. The firmware never
// touches it. Each SemgStimulusChannel is registered on an ADC channel and returns the column it was given, at the row the current
// virtual time falls in, as the ADC model samples it. The file is CSV with a
// header row and one column per electrode, in volts at the pin: the front
// end's mid-rail bias plus the amplified signal. Rows are spaced at
// SampleRateHz; past the last row the final row holds.
//
// The stimulus is synthetic. firmware/renode/stimulus/make_semg_stimulus.py
// generates it and says what it contains. No recorded sEMG is used anywhere in
// this repository.
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;

using Antmicro.Renode.Core;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Utilities.RESD;

namespace Antmicro.Renode.Peripherals.Analog
{
    public class SemgStimulus : IDoubleWordPeripheral, IKnownSize
    {
        public SemgStimulus(IMachine machine, double sampleRateHz = 2000.0, double restVolts = 1.5)
        {
            this.machine = machine;
            SampleRateHz = sampleRateHz;
            RestVolts = restVolts;
        }

        public void Reset()
        {
        }

        public uint ReadDoubleWord(long offset)
        {
            return offset == 0 ? (uint)Reads : 0;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
        }

        public long Size => 0x10;

        public void Load(string path)
        {
            var rows = new List<double[]>();
            var lines = File.ReadAllLines(path);
            for(var i = 1; i < lines.Length; i++)
            {
                var line = lines[i].Trim();
                if(line.Length == 0 || line.StartsWith("#"))
                {
                    continue;
                }
                var fields = line.Split(',');
                var row = new double[fields.Length];
                for(var j = 0; j < fields.Length; j++)
                {
                    row[j] = double.Parse(fields[j], CultureInfo.InvariantCulture);
                }
                rows.Add(row);
            }
            if(rows.Count == 0)
            {
                throw new RecoverableException($"{path} has no rows");
            }
            data = rows;
            this.Log(LogLevel.Info, "Loaded {0} rows of {1} channels from {2}", rows.Count, rows[0].Length, path);
        }

        public double SampleRateHz { get; set; }

        // What a channel reads with no file loaded, or for a column the file
        // does not have.
        public double RestVolts { get; set; }

        public ulong Reads { get; private set; }

        public double VoltsAt(int column)
        {
            Reads++;
            if(data == null)
            {
                return RestVolts;
            }
            if(machine.SystemBus.TryGetCurrentCPU(out var cpu))
            {
                cpu.SyncTime();
            }
            var seconds = machine.ClockSource.CurrentValue.TotalSeconds;
            var index = (long)Math.Floor(seconds * SampleRateHz);
            if(index < 0)
            {
                index = 0;
            }
            if(index >= data.Count)
            {
                index = data.Count - 1;
            }
            var row = data[(int)index];
            return column < row.Length ? row[column] : RestVolts;
        }

        private List<double[]> data;
        private readonly IMachine machine;
    }

    public class SemgStimulusChannel : IRESDSampleSource<VoltageSample>
    {
        public SemgStimulusChannel(SemgStimulus stimulus, int column)
        {
            this.stimulus = stimulus;
            this.column = column;
        }

        public void Reset()
        {
        }

        public VoltageSample Sample
        {
            get
            {
                var volts = stimulus.VoltsAt(column);
                if(volts < 0)
                {
                    volts = 0;
                }
                return new VoltageSample((uint)Math.Round(volts * 1e6));  // microvolts
            }
        }

        private readonly SemgStimulus stimulus;
        private readonly int column;
    }
}
