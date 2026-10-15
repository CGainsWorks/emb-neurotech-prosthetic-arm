#include "pwm_servo.hpp"

#include <algorithm>

#include "gpio.hpp"
#include "registers.hpp"
#include "system.hpp"

namespace pa::platform::pwm {
namespace {

bool g_enabled = false;

// One microsecond per tick. At a 200 MHz timer clock that is a prescaler of
// 200 and it makes the compare register the pulse width in microseconds with
// nothing to convert. A finer tick would buy resolution the servo cannot use:
// the class resolves about a microsecond.
constexpr uint32_t kTicksPerMicrosecond = 1;

uint32_t period_ticks() {
    return static_cast<uint32_t>(1'000'000.0f / config::kServoFrameRateHz);
}

void configure_timer(TimRegisters* timer, uint32_t channels) {
    timer->CR1 = 0;
    timer->PSC = (kTimerClockHz / 1'000'000u) / kTicksPerMicrosecond - 1;
    timer->ARR = period_ticks() - 1;

    // PWM mode 1 with the preload enabled on every channel in use, so a new
    // compare value takes effect at the next update rather than part way
    // through a pulse. Without the preload a pulse can be truncated mid-flight,
    // which the servo reads as a large step.
    timer->CCMR1 = (6u << 4) | (1u << 3) | (6u << 12) | (1u << 11);
    timer->CCMR2 = (6u << 4) | (1u << 3) | (6u << 12) | (1u << 11);
    timer->CCER = 0;
    for (uint32_t i = 0; i < channels; ++i) timer->CCR[i] = 0;

    timer->CR1 = (1u << 7);  // ARPE
    timer->EGR = 1u;         // load the prescaler and the preloads
}

}  // namespace

bool begin() {
    RCC->AHB4ENR |= (1u << 1) | (1u << 4);  // GPIOB, GPIOE
    RCC->APB2ENR |= (1u << 0);              // TIM1
    RCC->APB1LENR |= (1u << 1);             // TIM3
    dsb();

    // TIM1 channels 1 to 4 on PE9, PE11, PE13 and PE14, alternate function 1.
    //
    // Port E rather than the more obvious PA8 to PA11: PA9 and PA10 are
    // USART1, which is the link to the host and a pin cannot be a timer
    // output and a UART at the same time. Configuring both would leave
    // whichever ran second in charge, which on this board would be two servo
    // channels that silently never move. firmware/docs/PINOUT.md records the
    // conflict and this is where it is avoided.
    for (uint32_t pin : {9u, 11u, 13u, 14u})
        gpio::configure(GPIOE, pin, gpio::Mode::kAlternate, gpio::Pull::kDown,
                        gpio::Speed::kHigh, gpio::Type::kPushPull, 1);
    // TIM3 channels 1 and 2 on PB4 and PB5, alternate function 2.
    for (uint32_t pin = 4; pin <= 5; ++pin)
        gpio::configure(GPIOB, pin, gpio::Mode::kAlternate, gpio::Pull::kDown,
                        gpio::Speed::kHigh, gpio::Type::kPushPull, 2);

    configure_timer(TIM1, 4);
    configure_timer(TIM3, 2);

    // TIM1 is an advanced timer, so its outputs stay off until the main output
    // enable is set. That is a feature here: it means the pulse train cannot
    // start by accident before the safety layer has said so.
    TIM1->BDTR = (1u << 15);  // MOE

    // Both counters started from the same instruction, so the six pulses begin
    // together.
    TIM1->CNT = 0;
    TIM3->CNT = 0;
    TIM1->CR1 |= 1u;
    TIM3->CR1 |= 1u;

    disable();
    return true;
}

void set_pulses(const std::array<float, config::kServoChannels>& microseconds) {
    for (std::size_t i = 0; i < config::kServoChannels; ++i) {
        float pulse = microseconds[i];
        if (!(pulse > 0.0f)) {
            // Zero or not a number means no pulse at all, which is how the
            // driver asks for a relaxed servo.
            pulse = 0.0f;
        } else {
            pulse = std::max(config::kServoMinPulseUs,
                             std::min(config::kServoMaxPulseUs, pulse));
        }
        const uint32_t ticks = static_cast<uint32_t>(pulse * kTicksPerMicrosecond);
        if (i < 4) {
            TIM1->CCR[i] = ticks;
        } else {
            TIM3->CCR[i - 4] = ticks;
        }
    }
}

void enable() {
    TIM1->CCER = 0x1111u;  // CC1E to CC4E
    TIM3->CCER = 0x0011u;  // CC1E, CC2E
    g_enabled = true;
}

void disable() {
    TIM1->CCER = 0;
    TIM3->CCER = 0;
    g_enabled = false;
}

bool enabled() { return g_enabled; }

}  // namespace pa::platform::pwm
