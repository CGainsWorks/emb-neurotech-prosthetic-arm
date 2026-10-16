//
// Copyright (c) 2010-2025 Antmicro
// Copyright (c) 2026 CGainsWorks
//
// This file is licensed under the MIT License.
//
// An STM32H7 DMA controller (DMA1 or DMA2) for Renode, derived from Renode's
// own STM32DMA model. That model is written for the F4 and leaves three things
// out which this firmware depends on:
//
//   circular mode (CIRC)          the sEMG acquisition runs one circular
//                                 transfer for its whole life
//   half transfer (HTIF, HTIE)    the double buffer hands over a half at a
//                                 time, so the firmware never sees a frame
//                                 without it
//   EN cleared on completion      the UART driver restarts a stream only once
//                                 the hardware has released it
//
// Memory to peripheral transfers can also be paced: given a byte rate, the
// data is written to the peripheral at once but the transfer complete flag is
// raised only after the time the bytes would take on the wire. Without that a
// UART driven by DMA has infinite bandwidth and its overflow path can never be
// reached.
//
// Only what this firmware uses is modelled. FIFO mode, double buffer mode,
// bursts and the error flags are accepted and ignored.
using System.Collections.Generic;
using System.Linq;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.DMA
{
    public sealed class STM32H7_DMA : BasicDoubleWordPeripheral, IKnownSize, IGPIOReceiver, INumberedGPIOOutput
    {
        public STM32H7_DMA(IMachine machine, uint memoryToPeripheralBytesPerSecond = 0) : base(machine)
        {
            this.memoryToPeripheralBytesPerSecond = memoryToPeripheralBytesPerSecond;
            engine = new DmaEngine(machine.GetSystemBus(this));
            streams = Enumerable.Range(0, NumberOfStreams).Select(id => new Stream(this, machine, id)).ToArray();
            Connections = Enumerable.Range(0, NumberOfStreams).ToDictionary(id => id, id => (IGPIO)streams[id].IRQ);
            DefineRegisters();
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            foreach(var stream in streams)
            {
                stream.Reset();
            }
        }

        // A request line from the DMAMUX, one per stream.
        public void OnGPIO(int number, bool value)
        {
            if(number < 0 || number >= streams.Length)
            {
                this.WarningLog("Request for stream {0}, which does not exist", number);
                return;
            }
            if(value)
            {
                streams[number].OnRequest();
            }
        }

        public long Size => 0x400;

        public IReadOnlyDictionary<int, IGPIO> Connections { get; }

        // Completed transfers per stream, for tests.
        public ulong CompletedTransfers(int stream)
        {
            return streams[stream].Completed;
        }

        private void DefineRegisters()
        {
            // Flag positions within a status or clear register, for streams 0
            // to 3 (low) and 4 to 7 (high).
            var offsets = new[] { 0, 6, 16, 22 };

            var lowStatus = Registers.LowInterruptStatus.Define(this);
            var highStatus = Registers.HighInterruptStatus.Define(this);
            var lowClear = Registers.LowInterruptClear.Define(this);
            var highClear = Registers.HighInterruptClear.Define(this);

            for(var i = 0; i < NumberOfStreams; i++)
            {
                var stream = streams[i];
                var offset = offsets[i % 4];
                var status = i < 4 ? lowStatus : highStatus;
                var clear = i < 4 ? lowClear : highClear;

                status
                    .WithFlag(offset, FieldMode.Read, name: $"FEIF{i}", valueProviderCallback: _ => false)
                    .WithFlag(offset + 2, FieldMode.Read, name: $"DMEIF{i}", valueProviderCallback: _ => false)
                    .WithFlag(offset + 3, FieldMode.Read, name: $"TEIF{i}", valueProviderCallback: _ => false)
                    .WithFlag(offset + 4, FieldMode.Read, name: $"HTIF{i}", valueProviderCallback: _ => stream.HalfTransferFlag)
                    .WithFlag(offset + 5, FieldMode.Read, name: $"TCIF{i}", valueProviderCallback: _ => stream.TransferCompleteFlag);

                clear
                    .WithFlag(offset, FieldMode.Write, name: $"CFEIF{i}")
                    .WithFlag(offset + 2, FieldMode.Write, name: $"CDMEIF{i}")
                    .WithFlag(offset + 3, FieldMode.Write, name: $"CTEIF{i}")
                    .WithFlag(offset + 4, FieldMode.Write, name: $"CHTIF{i}",
                        writeCallback: (_, value) => { if(value) { stream.HalfTransferFlag = false; stream.UpdateInterrupt(); } })
                    .WithFlag(offset + 5, FieldMode.Write, name: $"CTCIF{i}",
                        writeCallback: (_, value) => { if(value) { stream.TransferCompleteFlag = false; stream.UpdateInterrupt(); } });
            }
        }

        private readonly Stream[] streams;
        private readonly DmaEngine engine;
        private readonly uint memoryToPeripheralBytesPerSecond;

        private const int NumberOfStreams = 8;
        private const int StreamStep = 0x18;

        private sealed class Stream
        {
            public Stream(STM32H7_DMA parent, IMachine machine, int id)
            {
                this.parent = parent;
                this.id = id;
                if(parent.memoryToPeripheralBytesPerSecond > 0)
                {
                    // One tick per microsecond is fine enough for a UART: a byte
                    // at 921600 baud is about eleven of them.
                    completionTimer = new LimitTimer(machine.ClockSource, 1000000, parent, $"stream{id}_pacing",
                        limit: 1, direction: Direction.Ascending, enabled: false, workMode: WorkMode.OneShot, eventEnabled: true);
                    completionTimer.LimitReached += Complete;
                }
                DefineRegisters();
            }

            public void Reset()
            {
                HalfTransferFlag = false;
                TransferCompleteFlag = false;
                dataOffset = 0;
                initialCount = 0;
                Completed = 0;
                completionTimer?.Reset();
                IRQ.Unset();
            }

            public void OnRequest()
            {
                if(!enabled.Value)
                {
                    // Real hardware ignores a request on a disabled stream too.
                    return;
                }
                if(direction.Value == TransferDirection.PeripheralToMemory)
                {
                    TransferOneItem();
                }
            }

            public void UpdateInterrupt()
            {
                var irq = (TransferCompleteFlag && transferCompleteInterruptEnable.Value)
                    || (HalfTransferFlag && halfTransferInterruptEnable.Value);
                IRQ.Set(irq);
            }

            public GPIO IRQ { get; } = new GPIO();

            public bool HalfTransferFlag { get; set; }

            public bool TransferCompleteFlag { get; set; }

            public ulong Completed { get; private set; }

            private void DefineRegisters()
            {
                var step = id * StreamStep;

                (Registers.StreamConfiguration + step).Define(parent)
                    .WithFlag(0, out enabled, name: "EN", writeCallback: (previous, value) => { if(value && !previous) OnEnable(); })
                    .WithFlag(1, name: "DMEIE")
                    .WithFlag(2, name: "TEIE")
                    .WithFlag(3, out halfTransferInterruptEnable, name: "HTIE", writeCallback: (_, __) => UpdateInterrupt())
                    .WithFlag(4, out transferCompleteInterruptEnable, name: "TCIE", writeCallback: (_, __) => UpdateInterrupt())
                    .WithFlag(5, name: "PFCTRL")
                    .WithEnumField(6, 2, out direction, name: "DIR")
                    .WithFlag(8, out circular, name: "CIRC")
                    .WithFlag(9, out peripheralIncrement, name: "PINC")
                    .WithFlag(10, out memoryIncrement, name: "MINC")
                    .WithEnumField(11, 2, out peripheralSize, name: "PSIZE")
                    .WithEnumField(13, 2, out memorySize, name: "MSIZE")
                    .WithFlag(15, name: "PINCOS")
                    .WithValueField(16, 2, name: "PL")
                    .WithFlag(18, name: "DBM")
                    .WithFlag(19, name: "CT")
                    .WithFlag(20, name: "TRBUFF")
                    .WithValueField(21, 2, name: "PBURST")
                    .WithValueField(23, 2, name: "MBURST")
                    .WithReservedBits(25, 7);

                (Registers.StreamNumberOfData + step).Define(parent)
                    .WithValueField(0, 16, out count, name: "NDT")
                    .WithReservedBits(16, 16);

                (Registers.StreamPeripheralAddress + step).Define(parent)
                    .WithValueField(0, 32, out peripheralAddress, name: "PAR");

                (Registers.StreamMemory0Address + step).Define(parent)
                    .WithValueField(0, 32, out memoryAddress, name: "M0AR");

                (Registers.StreamMemory1Address + step).Define(parent)
                    .WithValueField(0, 32, name: "M1AR");

                (Registers.StreamFIFOControl + step).Define(parent)
                    .WithValueField(0, 2, name: "FTH")
                    .WithFlag(2, name: "DMDIS")
                    .WithValueField(3, 3, FieldMode.Read, name: "FS", valueProviderCallback: _ => 4) // FIFO empty
                    .WithReservedBits(6, 1)
                    .WithFlag(7, name: "FEIE")
                    .WithReservedBits(8, 24);
            }

            private void OnEnable()
            {
                initialCount = count.Value;
                dataOffset = 0;
                if(initialCount == 0)
                {
                    parent.WarningLog("Stream {0} enabled with nothing to transfer", id);
                    return;
                }

                switch(direction.Value)
                {
                case TransferDirection.PeripheralToMemory:
                    // Paced by the peripheral's requests.
                    return;
                case TransferDirection.MemoryToMemory:
                    IssueBlock();
                    Complete();
                    return;
                case TransferDirection.MemoryToPeripheral:
                    IssueBlock();
                    if(completionTimer == null)
                    {
                        Complete();
                    }
                    else
                    {
                        // The rate is in bytes, so it already includes the start
                        // and stop bits: 921600 baud, 8N1, is 92160 bytes a second.
                        var bytes = (ulong)initialCount * (ulong)PeripheralBytes;
                        var microseconds = bytes * 1000000UL / parent.memoryToPeripheralBytesPerSecond;
                        completionTimer.Value = 0;
                        completionTimer.Limit = microseconds == 0 ? 1 : microseconds;
                        completionTimer.Enabled = true;
                    }
                    return;
                default:
                    parent.WarningLog("Stream {0} enabled with a reserved direction", id);
                    return;
                }
            }

            // Everything left in the stream, in one go. Used for the directions
            // that are not paced by peripheral requests.
            private void IssueBlock()
            {
                var source = direction.Value == TransferDirection.MemoryToPeripheral ? memoryAddress.Value : peripheralAddress.Value;
                var destination = direction.Value == TransferDirection.MemoryToPeripheral ? peripheralAddress.Value : memoryAddress.Value;
                var sourceIncrement = direction.Value == TransferDirection.MemoryToPeripheral ? memoryIncrement.Value : peripheralIncrement.Value;
                var destinationIncrement = direction.Value == TransferDirection.MemoryToPeripheral ? peripheralIncrement.Value : memoryIncrement.Value;
                var sourceType = TransferTypeOf(direction.Value == TransferDirection.MemoryToPeripheral ? memorySize.Value : peripheralSize.Value);
                var destinationType = TransferTypeOf(direction.Value == TransferDirection.MemoryToPeripheral ? peripheralSize.Value : memorySize.Value);
                var size = (int)count.Value * PeripheralBytes;

                parent.engine.IssueCopy(new Request(source, destination, size, sourceType, destinationType,
                    sourceIncrement, destinationIncrement));
                count.Value = 0;
            }

            // One data item from a peripheral to memory, on that peripheral's
            // request. Circular streams reload and raise the half and full
            // flags each time round, which is what the ADC double buffer runs on.
            private void TransferOneItem()
            {
                if(count.Value == 0)
                {
                    return;
                }

                var source = peripheralAddress.Value + (peripheralIncrement.Value ? dataOffset : 0);
                var destination = memoryAddress.Value + (memoryIncrement.Value ? dataOffset : 0);
                parent.engine.IssueCopy(new Request(source, destination, PeripheralBytes,
                    TransferTypeOf(peripheralSize.Value), TransferTypeOf(memorySize.Value), false, false));

                dataOffset += (ulong)PeripheralBytes;
                count.Value -= 1;

                if(count.Value == initialCount / 2 && initialCount > 1)
                {
                    HalfTransferFlag = true;
                    UpdateInterrupt();
                }
                if(count.Value == 0)
                {
                    Complete();
                }
            }

            private void Complete()
            {
                if(completionTimer != null)
                {
                    completionTimer.Enabled = false;
                }
                Completed++;
                TransferCompleteFlag = true;
                if(circular.Value && direction.Value != TransferDirection.MemoryToMemory)
                {
                    count.Value = initialCount;
                    dataOffset = 0;
                }
                else
                {
                    // Non-circular streams release themselves, which is what a
                    // driver polling EN before reprogramming relies on.
                    enabled.Value = false;
                }
                UpdateInterrupt();
            }

            private int PeripheralBytes => 1 << (int)peripheralSize.Value;

            private static TransferType TransferTypeOf(DataSize size)
            {
                switch(size)
                {
                case DataSize.HalfWord:
                    return TransferType.Word;
                case DataSize.Word:
                    return TransferType.DoubleWord;
                default:
                    return TransferType.Byte;
                }
            }

            private IFlagRegisterField enabled;
            private IFlagRegisterField halfTransferInterruptEnable;
            private IFlagRegisterField transferCompleteInterruptEnable;
            private IFlagRegisterField circular;
            private IFlagRegisterField peripheralIncrement;
            private IFlagRegisterField memoryIncrement;
            private IEnumRegisterField<TransferDirection> direction;
            private IEnumRegisterField<DataSize> peripheralSize;
            private IEnumRegisterField<DataSize> memorySize;
            private IValueRegisterField count;
            private IValueRegisterField peripheralAddress;
            private IValueRegisterField memoryAddress;

            private ulong dataOffset;
            private ulong initialCount;

            private readonly LimitTimer completionTimer;
            private readonly STM32H7_DMA parent;
            private readonly int id;
        }

        private enum TransferDirection
        {
            PeripheralToMemory = 0,
            MemoryToPeripheral = 1,
            MemoryToMemory = 2,
            Reserved = 3,
        }

        private enum DataSize
        {
            Byte = 0,
            HalfWord = 1,
            Word = 2,
            Reserved = 3,
        }

        private enum Registers
        {
            LowInterruptStatus = 0x0,
            HighInterruptStatus = 0x4,
            LowInterruptClear = 0x8,
            HighInterruptClear = 0xC,
            StreamConfiguration = 0x10,
            StreamNumberOfData = 0x14,
            StreamPeripheralAddress = 0x18,
            StreamMemory0Address = 0x1C,
            StreamMemory1Address = 0x20,
            StreamFIFOControl = 0x24,
        }
    }
}
