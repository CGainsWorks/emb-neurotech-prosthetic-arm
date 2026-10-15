#include "adc.hpp"

#include "gpio.hpp"
#include "registers.hpp"
#include "system.hpp"

namespace pa::platform::adc {
namespace {

// The eight differential pairs, as ADC1 channel numbers. PINOUT.md maps these
// to pins and to electrode positions.
constexpr uint32_t kChannels[config::kSemgChannels] = {3, 7, 10, 11, 14, 15, 16, 18};

// Two sets, so the DMA can be writing one while the application reads the
// other. Placed by the linker in AXI SRAM: the DMA controller cannot reach the
// tightly coupled RAM the stack lives in and a buffer that the hardware
// cannot write to is a silent failure rather than a loud one.
__attribute__((section(".dma_buffer"), aligned(32)))
volatile uint16_t g_samples[2][config::kSemgChannels];

volatile uint32_t g_ready_half = 0;
volatile bool g_has_new = false;
volatile uint32_t g_dropped = 0;
volatile uint32_t g_timestamp_us = 0;

bool wait_for_bit(volatile uint32_t& reg, uint32_t mask, bool set, uint32_t spins) {
    for (uint32_t i = 0; i < spins; ++i) {
        if (((reg & mask) != 0) == set) return true;
    }
    return false;
}

void configure_trigger_timer() {
    // TIM2 free-running at the acquisition rate, with its update event routed
    // out as a trigger. Nothing else uses TIM2.
    RCC->APB1LENR |= (1u << 0);
    dsb();

    const uint32_t ticks = static_cast<uint32_t>(
        static_cast<float>(kTimerClockHz) / config::kSemgSampleRateHz);
    TIM2->PSC = 0;
    TIM2->ARR = ticks - 1;
    TIM2->CR2 = (2u << 4);  // MMS = update event as TRGO
    TIM2->CR1 = 1u;         // CEN
}

void configure_dma() {
    RCC->AHB1ENR |= (1u << 0);  // DMA1
    dsb();

    // Request 9 on DMAMUX1 is ADC1 (RM0433 table 121).
    DMAMUX1[0].CCR = 9u;

    DmaStreamRegisters& stream = DMA1->stream[0];
    stream.CR = 0;
    while (stream.CR & 1u) {
    }
    stream.PAR = reinterpret_cast<uint32_t>(&ADC1->DR);
    stream.M0AR = reinterpret_cast<uint32_t>(&g_samples[0][0]);
    stream.NDTR = 2 * config::kSemgChannels;
    stream.FCR = 0;  // direct mode, no FIFO: transfers are half words and regular
    stream.CR = (0u << 6)     // peripheral to memory
              | (1u << 8)     // circular
              | (1u << 10)    // memory increment
              | (1u << 11)    // peripheral data size: half word
              | (1u << 13)    // memory data size: half word
              | (3u << 16)    // very high priority
              | (1u << 3)     // half transfer interrupt
              | (1u << 4);    // transfer complete interrupt
    stream.CR |= 1u;          // enable

    nvic_enable(kDma1Stream0, 1);
}

}  // namespace

bool begin() {
    // Analogue pins. Nothing to configure beyond the mode: in analogue mode
    // the Schmitt trigger and both pull resistors are already disconnected,
    // which is what keeps the input impedance where the front end assumes it.
    RCC->AHB4ENR |= 0x1Fu;  // GPIOA through GPIOE
    dsb();
    for (uint32_t pin = 0; pin < 8; ++pin)
        gpio::configure(GPIOA, pin, gpio::Mode::kAnalogue);
    for (uint32_t pin = 0; pin < 2; ++pin)
        gpio::configure(GPIOC, pin, gpio::Mode::kAnalogue);

    RCC->AHB1ENR |= (1u << 5);  // ADC1 and ADC2
    dsb();

    // Leave deep power down, then start the internal regulator and give it the
    // 10 microseconds the datasheet asks for.
    ADC1->CR &= ~(1u << 29);  // DEEPPWD
    ADC1->CR |= (1u << 28);   // ADVREGEN
    delay_us(20);

    // Clock the ADC from the AHB bus divided by four: 50 MHz, inside the
    // 50 MHz maximum for a 16-bit conversion at VOS1.
    ADC12_COMMON->CCR = (0u << 16)   // CKMODE handled below
                      | (3u << 18);  // PRESC = 4
    ADC12_COMMON->CCR |= (3u << 16); // CKMODE = AHB/4

    // Boost mode, needed whenever the ADC clock is above 20 MHz.
    ADC1->CR |= (3u << 8);

    // Single-ended calibration, then differential. Both are needed because the
    // front end presents a differential pair per electrode.
    ADC1->CR &= ~(1u << 30);  // ADCALDIF: single ended
    ADC1->CR |= (1u << 16);   // ADCALLIN: linearity too
    ADC1->CR |= (1u << 31);   // ADCAL
    if (!wait_for_bit(ADC1->CR, 1u << 31, false, 1'000'000)) return false;

    ADC1->CR |= (1u << 30);   // ADCALDIF: differential
    ADC1->CR |= (1u << 31);
    if (!wait_for_bit(ADC1->CR, 1u << 31, false, 1'000'000)) return false;
    ADC1->CR &= ~(1u << 30);

    configure_dma();

    // 16-bit, hardware triggered on TIM2 TRGO, rising edge, DMA in circular
    // mode. Overrun is left to overwrite rather than to stop: a converter that
    // halts on the first missed read takes the arm with it and the drop
    // counter reports the same fact without doing that.
    ADC1->CFGR = (0u << 2)     // RES: 16 bit
               | (1u << 10)    // EXTEN: rising edge
               | (18u << 5)    // EXTSEL: TIM2 TRGO (RM0433 table 210)
               | (1u << 12)    // OVRMOD: overwrite
               | (1u << 0)     // DMNGT: DMA circular
               | (1u << 1);

    // Sampling time. 32.5 ADC cycles at 50 MHz is 650 ns per channel, which an
    // op-amp driving a 10 pF sampling capacitor settles inside comfortably;
    // eight of them plus conversion fits the 500 microsecond period with two
    // orders of magnitude to spare.
    ADC1->SMPR1 = 0;
    ADC1->SMPR2 = 0;
    for (std::size_t i = 0; i < config::kSemgChannels; ++i) {
        const uint32_t channel = kChannels[i];
        if (channel < 10) {
            ADC1->SMPR1 |= (3u << (channel * 3));
        } else {
            ADC1->SMPR2 |= (3u << ((channel - 10) * 3));
        }
        ADC1->PCSEL |= (1u << channel);
    }

    // The regular sequence: eight conversions, in electrode order.
    ADC1->SQR1 = config::kSemgChannels - 1;
    for (std::size_t i = 0; i < config::kSemgChannels; ++i) {
        const uint32_t channel = kChannels[i];
        if (i < 4) {
            ADC1->SQR1 |= (channel << (6 * (i + 1)));
        } else if (i < 9) {
            ADC1->SQR2 |= (channel << (6 * (i - 4)));
        }
    }

    ADC1->ISR = 0x7FF;      // clear everything
    ADC1->CR |= (1u << 0);  // ADEN
    if (!wait_for_bit(ADC1->ISR, 1u << 0, true, 1'000'000)) return false;  // ADRDY

    configure_trigger_timer();
    ADC1->CR |= (1u << 2);  // ADSTART

    return true;
}

bool read(SampleSet& out, uint32_t& timestamp_us) {
    if (!g_has_new) return false;
    const uint32_t half = g_ready_half;
    for (std::size_t i = 0; i < config::kSemgChannels; ++i) {
        // The converter is unipolar and the front end sits at mid-rail, so the
        // conversion to a signed sample is a subtraction of half scale. The
        // shift makes it a 16-bit signed value at the same scale the rest of
        // the firmware works in.
        const int32_t raw = static_cast<int32_t>(g_samples[half][i]);
        out[i] = static_cast<int16_t>(raw - 32768);
    }
    timestamp_us = g_timestamp_us;
    g_has_new = false;
    return true;
}

uint32_t dropped() { return g_dropped; }

extern "C" void DMA1_Stream0_IRQHandler() {
    const uint32_t status = DMA1->LISR;
    // Half transfer: the first set is complete. Transfer complete: the second.
    if (status & (1u << 4)) {
        DMA1->LIFCR = (1u << 4);
        if (g_has_new) ++g_dropped;
        g_ready_half = 0;
        g_timestamp_us = micros();
        g_has_new = true;
    }
    if (status & (1u << 5)) {
        DMA1->LIFCR = (1u << 5);
        if (g_has_new) ++g_dropped;
        g_ready_half = 1;
        g_timestamp_us = micros();
        g_has_new = true;
    }
}

}  // namespace pa::platform::adc
