// Reset handler and vector table.
//
// Replaces the vendor startup assembly. Everything it does is in C++ and
// visible: copy the initialised data out of flash, zero the rest, turn on the
// floating point unit, run the static constructors and call main.
// -nostartfiles in the link means nothing else runs first.
//
// It has never executed on silicon. It does run under Renode, on an emulated
// STM32H743 (see firmware/README.md), which is how the missing constructor
// call below was found.
#include <array>
#include <cstdint>

#include "registers.hpp"

extern "C" {

// Provided by the linker script.
extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss;
extern uint32_t _sdma, _edma;

// The static constructors, also placed by the linker script. -nostartfiles
// removes the C runtime that would normally walk these.
using Constructor = void (*)();
extern Constructor __preinit_array_start[], __preinit_array_end[];
extern Constructor __init_array_start[], __init_array_end[];

// The initial stack pointer, declared as a function so that its address is a
// constant expression and the vector table can be built at compile time. It is
// not a function and is never called; the linker script assigns it the top of
// the tightly coupled RAM.
void _estack_symbol();

int app_main();

void Reset_Handler();
void Default_Handler();

// The three handlers that are actually implemented. Everything else falls
// through to the default one, which stops rather than returning into whatever
// was interrupted.
void DMA1_Stream0_IRQHandler();
void DMA1_Stream1_IRQHandler();
void USART1_IRQHandler();

void NMI_Handler() { while (true) {} }
void MemManage_Handler() { while (true) {} }
void BusFault_Handler() { while (true) {} }
void UsageFault_Handler() { while (true) {} }
void SVC_Handler() {}
void DebugMon_Handler() {}
void PendSV_Handler() {}
void SysTick_Handler() {}

// Taking the outputs down is the first thing any unrecoverable handler does.
// The servo timers run from their own clock and would keep producing pulses
// long after the core stopped, so a fault that only halted the CPU would leave
// an arm holding whatever position it was in, indefinitely, with nothing
// watching it.
void stop_outputs() {
    pa::platform::TIM1->BDTR = 0;
    pa::platform::TIM1->CCER = 0;
    pa::platform::TIM3->CCER = 0;
}

void HardFault_Handler() {
    stop_outputs();
    while (true) {
    }
}

void Default_Handler() {
    stop_outputs();
    while (true) {
    }
}

using Handler = void (*)();

// 150 external interrupt lines on this part, plus the sixteen core entries.
// Entry 16 + n is external interrupt n, so the DMA1 stream 0 handler at
// interrupt 11 sits at index 27.
//
// Built by a constexpr function rather than written out as 166 initialisers.
// The array is still a compile-time constant and still lands in flash; what it
// avoids is 150 lines of Default_Handler, in which a single misplaced entry
// would be invisible.
constexpr std::size_t kVectorCount = 166;

constexpr std::array<Handler, kVectorCount> make_vectors() {
    std::array<Handler, kVectorCount> vectors{};
    for (std::size_t i = 16; i < kVectorCount; ++i) vectors[i] = Default_Handler;

    vectors[0] = _estack_symbol;
    vectors[1] = Reset_Handler;
    vectors[2] = NMI_Handler;
    vectors[3] = HardFault_Handler;
    vectors[4] = MemManage_Handler;
    vectors[5] = BusFault_Handler;
    vectors[6] = UsageFault_Handler;
    vectors[11] = SVC_Handler;
    vectors[12] = DebugMon_Handler;
    vectors[14] = PendSV_Handler;
    vectors[15] = SysTick_Handler;

    vectors[16 + 11] = DMA1_Stream0_IRQHandler;
    vectors[16 + 12] = DMA1_Stream1_IRQHandler;
    vectors[16 + 37] = USART1_IRQHandler;
    return vectors;
}

__attribute__((section(".isr_vector"), used))
const std::array<Handler, kVectorCount> g_vectors = make_vectors();

void Reset_Handler() {
    // The floating point unit, before anything that might use it. The compiler
    // emits VFP instructions for the filter design at start-up and a hard
    // fault from an unenabled coprocessor is an unhelpful way to find that out.
    pa::platform::SCB->CPACR |= (0xFu << 20);
    pa::platform::dsb();
    pa::platform::isb();

    // Initialised data, out of flash and into RAM.
    uint32_t* source = &_sidata;
    for (uint32_t* destination = &_sdata; destination < &_edata;) *destination++ = *source++;

    // Everything else, zeroed.
    for (uint32_t* destination = &_sbss; destination < &_ebss;) *destination++ = 0;

    // The DMA buffers have a section of their own because they have to live in
    // AXI SRAM rather than in the tightly coupled RAM the rest of the data is
    // in: the DMA controller has no path to DTCM and a buffer the hardware
    // cannot reach fails silently rather than loudly.
    for (uint32_t* destination = &_sdma; destination < &_edma;) *destination++ = 0;

    // Static constructors, after the data they write to exists and before
    // anything uses the objects they build. Every global in main.cpp with a
    // virtual function or a non-constant initialiser is built here: without
    // this loop the I2C bus object has no vtable and the first call through it,
    // the IMU's WHO_AM_I read, jumps to address zero. The first boot under
    // Renode stopped exactly there with an INVSTATE usage fault.
    for (Constructor* constructor = __preinit_array_start; constructor < __preinit_array_end;
         ++constructor)
        (*constructor)();
    for (Constructor* constructor = __init_array_start; constructor < __init_array_end;
         ++constructor)
        (*constructor)();

    app_main();

    // It does not return. If it ever does, stop with the outputs off rather
    // than running off the end of flash.
    Default_Handler();
}

}  // extern "C"
