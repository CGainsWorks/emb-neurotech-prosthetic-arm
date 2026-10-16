//
// Copyright (c) 2026 CGainsWorks
//
// This file is licensed under the MIT License.
//
// An LSM6DSOX inertial module on I2C, enough of it for the firmware's driver.
//
// Renode ships an LSM6DSO model but it does not implement STATUS_REG (0x1E),
// which is the register this firmware polls before every read. Against that
// model the driver would see "no new data" forever and the orientation filter
// would only ever coast. This one implements the registers the driver touches
// and nothing else:
//
//   WHO_AM_I 0x0F      0x6C
//   CTRL1_XL 0x10      output rate and full scale of the accelerometer
//   CTRL2_G  0x11      output rate and full scale of the gyroscope
//   CTRL3_C  0x12      IF_INC is honoured; BDU is accepted (reads here are
//                      always coherent)
//   STATUS   0x1E      XLDA, GDA and TDA set while the matching output rate
//                      is not zero
//   OUT_TEMP 0x20-0x21 256 counts per degree, zero at 25 C
//   OUTX_G   0x22-0x27 scaled by the configured full scale
//   OUTX_A   0x28-0x2D scaled by the configured full scale
//
// The readings are whatever the test sets through the properties below, in SI
// units. They are a stimulus, not a model of a sensor's noise or bias.
using System;
using System.Collections.Generic;

using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.I2C;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class LSM6DSOX : II2CPeripheral
    {
        public LSM6DSOX(IMachine machine)
        {
            Reset();
        }

        public void Reset()
        {
            registers.Clear();
            pointer = 0;
            registers[Ctrl3C] = 0x04;  // IF_INC is set out of reset
        }

        public void Write(byte[] data)
        {
            if(data.Length == 0)
            {
                return;
            }
            pointer = data[0];
            for(var i = 1; i < data.Length; i++)
            {
                WriteRegister(pointer, data[i]);
                Advance();
            }
        }

        public byte[] Read(int count = 1)
        {
            var result = new byte[count];
            for(var i = 0; i < count; i++)
            {
                result[i] = ReadRegister(pointer);
                Advance();
            }
            return result;
        }

        public void FinishTransmission()
        {
        }

        // Metres per second squared, in the sensor's own axes.
        public double AccelerationX { get; set; }
        public double AccelerationY { get; set; }
        public double AccelerationZ { get; set; } = StandardGravity;

        // Radians per second.
        public double AngularRateX { get; set; }
        public double AngularRateY { get; set; }
        public double AngularRateZ { get; set; }

        // Degrees Celsius, the die temperature.
        public double Temperature { get; set; } = 25.0;

        // How many times each register group has been read, for tests that
        // want to know the driver is polling at the rate it says.
        public ulong StatusReads { get; private set; }
        public ulong SampleReads { get; private set; }

        private void Advance()
        {
            if((Get(Ctrl3C) & 0x04) != 0)
            {
                pointer = (byte)(pointer + 1);
            }
        }

        private void WriteRegister(byte address, byte value)
        {
            switch(address)
            {
            case Ctrl1Xl:
            case Ctrl2G:
            case Ctrl3C:
                registers[address] = value;
                break;
            default:
                this.Log(LogLevel.Warning, "Write of 0x{0:X2} to register 0x{1:X2}, which is not modelled", value, address);
                break;
            }
        }

        private byte ReadRegister(byte address)
        {
            switch(address)
            {
            case WhoAmI:
                return 0x6C;
            case Ctrl1Xl:
            case Ctrl2G:
            case Ctrl3C:
                return Get(address);
            case Status:
                StatusReads++;
                var status = 0x04;  // TDA: temperature is always available
                if((Get(Ctrl1Xl) >> 4) != 0)
                {
                    status |= 0x01;  // XLDA
                }
                if((Get(Ctrl2G) >> 4) != 0)
                {
                    status |= 0x02;  // GDA
                }
                return (byte)status;
            case 0x20:
            case 0x21:
                return Half(Saturate((Temperature - 25.0) * 256.0), address - 0x20);
            case 0x22:
            case 0x23:
                SampleReads++;
                return Half(Saturate(AngularRateX / GyroRadiansPerCount()), address - 0x22);
            case 0x24:
            case 0x25:
                return Half(Saturate(AngularRateY / GyroRadiansPerCount()), address - 0x24);
            case 0x26:
            case 0x27:
                return Half(Saturate(AngularRateZ / GyroRadiansPerCount()), address - 0x26);
            case 0x28:
            case 0x29:
                return Half(Saturate(AccelerationX / AccelMetresPerCount()), address - 0x28);
            case 0x2A:
            case 0x2B:
                return Half(Saturate(AccelerationY / AccelMetresPerCount()), address - 0x2A);
            case 0x2C:
            case 0x2D:
                return Half(Saturate(AccelerationZ / AccelMetresPerCount()), address - 0x2C);
            default:
                this.Log(LogLevel.Warning, "Read of register 0x{0:X2}, which is not modelled", address);
                return 0;
            }
        }

        // Datasheet table 2: sensitivity by full-scale selection.
        private double AccelMetresPerCount()
        {
            double milliG;
            switch((Get(Ctrl1Xl) >> 2) & 0x3)
            {
            case 0: milliG = 0.061; break;   // 2 g
            case 1: milliG = 0.488; break;   // 16 g
            case 2: milliG = 0.122; break;   // 4 g
            default: milliG = 0.244; break;  // 8 g
            }
            return milliG * 1e-3 * StandardGravity;
        }

        private double GyroRadiansPerCount()
        {
            double milliDps;
            if((Get(Ctrl2G) & 0x02) != 0)
            {
                milliDps = 4.375;  // 125 dps
            }
            else
            {
                switch((Get(Ctrl2G) >> 2) & 0x3)
                {
                case 0: milliDps = 8.75; break;   // 250 dps
                case 1: milliDps = 17.5; break;   // 500 dps
                case 2: milliDps = 35.0; break;   // 1000 dps
                default: milliDps = 70.0; break;  // 2000 dps
                }
            }
            return milliDps * 1e-3 * Math.PI / 180.0;
        }

        private static short Saturate(double counts)
        {
            var rounded = Math.Round(counts);
            if(rounded > short.MaxValue)
            {
                return short.MaxValue;
            }
            if(rounded < short.MinValue)
            {
                return short.MinValue;
            }
            return (short)rounded;
        }

        private static byte Half(short value, int index)
        {
            return index == 0 ? (byte)(value & 0xFF) : (byte)((value >> 8) & 0xFF);
        }

        private byte Get(byte address)
        {
            return registers.TryGetValue(address, out var value) ? value : (byte)0;
        }

        private byte pointer;
        private readonly Dictionary<byte, byte> registers = new Dictionary<byte, byte>();

        private const double StandardGravity = 9.80665;
        private const byte WhoAmI = 0x0F;
        private const byte Ctrl1Xl = 0x10;
        private const byte Ctrl2G = 0x11;
        private const byte Ctrl3C = 0x12;
        private const byte Status = 0x1E;
    }
}
