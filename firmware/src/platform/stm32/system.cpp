#include "system.hpp"

#include "registers.hpp"

namespace pa::platform {
namespace {

// Roughly a millisecond of spinning at 64 MHz, used only while waiting for an
// oscillator to start. Long enough that a dead crystal is reported rather than
// hung on, short enough that a healthy one is never rejected: a 25 MHz crystal
// with a 2 ms start-up specification is inside this by an order of magnitude.
constexpr uint32_t kOscillatorTimeout = 400'000;

bool wait_for(volatile uint32_t& reg, uint32_t mask, bool set) {
    for (uint32_t spin = 0; spin < kOscillatorTimeout; ++spin) {
        const bool state = (reg & mask) != 0;
        if (state == set) return true;
    }
    return false;
}

uint32_t cycles_per_microsecond = 64;  // until the PLL is up

}  // namespace

bool configure_clocks() {
    // 1. Voltage scale 1. The flash wait states and the maximum bus frequency
    //    both depend on it, so it goes first and the regulator is given time
    //    to settle before anything is asked to run faster.
    PWR->CR3 = (PWR->CR3 & ~(1u << 2)) | (1u << 1);  // LDO enabled, SMPS off
    PWR->D3CR = (PWR->D3CR & ~(3u << 14)) | (3u << 14);  // VOS1
    if (!wait_for(PWR->D3CR, 1u << 13, true)) return false;  // VOSRDY

    // 2. Start the 25 MHz crystal.
    RCC->CR |= (1u << 16);  // HSEON
    if (!wait_for(RCC->CR, 1u << 17, true)) return false;  // HSERDY

    // 3. PLL1 from HSE: 25 MHz / 5 = 5 MHz reference, times 160 = 800 MHz in
    //    the VCO, divided by 2 = 400 MHz system clock. The reference has to
    //    land in one of the four RGE bands and 5 MHz is in the 4 to 8 MHz one.
    RCC->PLLCKSELR = (5u << 4) | 2u;  // DIVM1 = 5, PLLSRC = HSE
    RCC->PLLCFGR = (2u << 2)     // PLL1RGE: 4 to 8 MHz
                 | (0u << 1)     // PLL1VCOSEL: wide range, 192 to 836 MHz
                 | (1u << 16);   // DIVP1EN
    // The DIVR fields are all "value minus one".
    RCC->PLL1DIVR = (159u << 0)  // DIVN1 = 160
                  | (1u << 9)    // DIVP1 = 2
                  | (3u << 16)   // DIVQ1 = 4, spare
                  | (3u << 24);  // DIVR1 = 4, spare
    RCC->CR |= (1u << 24);       // PLL1ON
    if (!wait_for(RCC->CR, 1u << 25, true)) return false;  // PLL1RDY

    // 4. Bus prescalers, set before the switch so nothing is ever overclocked
    //    for the handful of cycles in between.
    RCC->D1CFGR = (0u << 8)   // D1CPRE = 1, so the core sees 400 MHz
                | (8u << 0);  // HPRE = 2, so AHB is 200 MHz
    RCC->D2CFGR = (4u << 4) | (4u << 8);   // APB1 and APB2 divided by two
    RCC->D3CFGR = (4u << 4);               // APB4 divided by two

    // 5. Flash latency for 200 MHz of AXI at VOS1: four wait states and the
    //    programming delay that goes with them (RM0433 table 17).
    FLASH->ACR = (2u << 4) | 4u;
    if ((FLASH->ACR & 0xFu) != 4u) return false;

    // 6. Switch.
    RCC->CFGR = (RCC->CFGR & ~7u) | 3u;  // SW = PLL1
    if (!wait_for(RCC->CFGR, 7u << 3, true)) return false;
    if (((RCC->CFGR >> 3) & 7u) != 3u) return false;

    cycles_per_microsecond = kSysClockHz / 1'000'000u;

    // 7. The cycle counter. It lives in the debug unit and needs the trace
    //    block enabled, which is free and does not require a debugger attached.
    COREDEBUG->DEMCR |= (1u << 24);  // TRCENA
    DWT->CYCCNT = 0;
    DWT->CTRL |= 1u;  // CYCCNTENA

    return true;
}

uint32_t micros() {
    return DWT->CYCCNT / cycles_per_microsecond;
}

uint32_t millis() {
    return DWT->CYCCNT / (cycles_per_microsecond * 1000u);
}

void delay_us(uint32_t microseconds) {
    const uint32_t start = DWT->CYCCNT;
    const uint32_t target = microseconds * cycles_per_microsecond;
    // Subtraction in the counter's own width, so this is correct across the
    // wrap that happens every ten seconds at 400 MHz.
    while ((DWT->CYCCNT - start) < target) {
    }
}

void delay_ms(uint32_t milliseconds) {
    for (uint32_t i = 0; i < milliseconds; ++i) delay_us(1000);
}

}  // namespace pa::platform
