# Pinout

STM32H743VIT6, 100-pin LQFP.

> **A design, not a netlist extracted from a board.** No schematic has been
> captured and no board has been fabricated, so this is the pin assignment the
> firmware is written against rather than a record of what was built. The
> peripheral and alternate-function choices are from the datasheet's
> alternate-function table; the physical placement is not fixed.

## sEMG acquisition

Eight differential channels into ADC1, scanned in order, triggered by TIM2 at
2 kHz and delivered by DMA1 stream 0.

| Channel | ADC input | Pin | Electrode position |
|---|---|---|---|
| 0 | ADC1_INP3 | PA6 | flexor carpi radialis |
| 1 | ADC1_INP7 | PA7 | flexor carpi ulnaris |
| 2 | ADC1_INP10 | PC0 | flexor digitorum superficialis |
| 3 | ADC1_INP11 | PC1 | palmaris longus |
| 4 | ADC1_INP14 | PA2 | extensor carpi radialis longus |
| 5 | ADC1_INP15 | PA3 | extensor digitorum |
| 6 | ADC1_INP16 | PA0 | extensor carpi ulnaris |
| 7 | ADC1_INP18 | PA4 | brachioradialis |

The muscle names are the intended electrode sites for a forearm band. They are
where the electrodes are meant to go, not a claim that any signal has been
recorded from them.

Analogue mode disconnects the Schmitt trigger and both pull resistors, which
is what keeps the input impedance where the front end assumes it.

### Why these channels

ADC1 on this part can reach inputs 0 to 19 and the eight above are the ones
whose pins are not wanted by the servo timers, the UART or the I2C bus. There
is nothing clever in the choice beyond that.

## Servo outputs

Six channels at 333 Hz, split across two timers because no timer on this part
has six compare channels. Both are started from the same instruction so the
pulses stay in phase, which is what makes the current draw predictable.

| Joint | Timer | Channel | Pin | AF |
|---|---|---|---|---|
| shoulder rotation | TIM1 | CH1 | PE9 | AF1 |
| shoulder flexion | TIM1 | CH2 | PE11 | AF1 |
| elbow flexion | TIM1 | CH3 | PE13 | AF1 |
| forearm rotation | TIM1 | CH4 | PE14 | AF1 |
| wrist flexion | TIM3 | CH1 | PB4 | AF2 |
| tool roll | TIM3 | CH2 | PB5 | AF2 |

TIM1 is an advanced timer, so its outputs stay off until the main output
enable is set. That is useful here rather than inconvenient: the pulse train
cannot start by accident before the safety layer has said so.

TIM1's channels are on port E rather than on the more obvious PA8 to PA11
and that is the one interesting thing about this table. PA9 and PA10 are
USART1, the link to the host; a pin cannot be a timer output and a UART at the
same time and configuring both would leave whichever ran second in charge.
The symptom would be two servo channels that silently never move, which is a
day on a bench to find.

Port E has the same alternate function available and nothing else on this
board wants it.

## Link to the host

| Signal | Pin | AF | Notes |
|---|---|---|---|
| USART1_TX | PA9 | AF7 | which is why the servo outputs are on port E |
| USART1_RX | PA10 | AF7 | |

921600 baud, 8N1, no flow control. Transmit by DMA1 stream 1 out of a 4 kB
ring; receive by interrupt, a byte at a time, straight into the frame decoder.

## IMU

| Signal | Pin | AF | Notes |
|---|---|---|---|
| I2C1_SCL | PB6 | AF4 | open drain, 2.2 kΩ pull-up to 3.3 V |
| I2C1_SDA | PB7 | AF4 | open drain, 2.2 kΩ pull-up to 3.3 V |

LSM6DSOX at address 0x6A with SDO tied low. 400 kHz fast mode.

The pull-ups are on the board rather than internal: the internal ones are tens
of kilohms and will not meet the rise time at 400 kHz with any real bus
capacitance.

## Current sense

| Signal | Pin | Notes |
|---|---|---|
| ADC1_INP5 | PB1 | INA181A2 output, 5 mΩ shunt in the servo return |

Not read by the firmware yet. The fault layer trips on the estimate in
`pa::servo::estimate_current` until this is measured;
[`BRING_UP.md`](BRING_UP.md) step 18 is where the two are compared.

## Debug

| Signal | Pin |
|---|---|
| SWDIO | PA13 |
| SWCLK | PA14 |

Nothing here is isolated. A probe attached to a mains-powered laptop puts
anything touching the electrodes and mains earth in the same circuit. See
[`docs/SAFETY.md`](../../docs/SAFETY.md).

## Interrupts the firmware installs

| Vector | Interrupt | Priority | Handler |
|---|---|---|---|
| 27 | DMA1 stream 0 | 1 | acquisition buffer complete |
| 28 | DMA1 stream 1 | 2 | transmit burst complete |
| 53 | USART1 | 2 | one received byte into the decoder |

Everything else falls through to the default handler, which takes the servo
outputs down and halts. `./firmware/verify_image.sh` checks all three are at
the right index in the vector table, which is the sort of thing that otherwise
produces a board that runs and quietly never services its DMA.

## Power

| Rail | Source | Load |
|---|---|---|
| 6.0 V | TPS568230 buck from the pack | six servos |
| 3.3 V | LMR33620 buck from the pack | STM32H743, IMU, INA181 |
| 3.0 V | TPS7A4700 LDO from 3.3 V | analogue front end, ADC reference |

The analogue rail comes from an LDO rather than straight from the buck
because, at a gain of 306, a millivolt of switching ripple on the reference is
3 µV at the electrode, which is three times the whole noise budget.
[`thermal_budget.md`](../../hardware/semg-servo-board/docs/thermal_budget.md)
has the tree and the losses.
