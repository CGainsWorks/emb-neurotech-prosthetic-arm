// STM32H743 peripheral registers, only the ones this firmware touches.
//
// Hand-written rather than pulled from the vendor headers. The device header
// is 50,000 lines of macros and dragging it in would mean dragging in the HAL
// behind it; what is actually needed here is nine peripherals and writing
// them out makes every register access something a reader can check against
// the reference manual (RM0433) rather than something hidden behind a name.
//
// None of this has ever been executed. No board exists.
#pragma once

#include <cstdint>

namespace pa::platform {

using reg32 = volatile uint32_t;

// --- Cortex-M7 core ---------------------------------------------------------
struct SysTickRegisters {
    reg32 CTRL, LOAD, VAL, CALIB;
};
struct ScbRegisters {
    reg32 CPUID, ICSR, VTOR, AIRCR, SCR, CCR;
    reg32 SHPR[3];
    reg32 SHCSR, CFSR, HFSR, DFSR, MMFAR, BFAR, AFSR;
    reg32 reserved0[18];
    reg32 CPACR;
};
struct NvicRegisters {
    reg32 ISER[8];
    reg32 reserved0[24];
    reg32 ICER[8];
    reg32 reserved1[24];
    reg32 ISPR[8];
    reg32 reserved2[24];
    reg32 ICPR[8];
    reg32 reserved3[24];
    reg32 IABR[8];
    reg32 reserved4[56];
    volatile uint8_t IPR[240];
};
// Data watchpoint cycle counter. The microsecond clock the scheduler and the
// timing statistics run on: free, one cycle to read and no timer burned on it.
struct DwtRegisters {
    reg32 CTRL, CYCCNT;
};
struct CoreDebugRegisters {
    reg32 DHCSR, DCRSR, DCRDR, DEMCR;
};

inline auto* const SYSTICK = reinterpret_cast<SysTickRegisters*>(0xE000E010u);
inline auto* const SCB = reinterpret_cast<ScbRegisters*>(0xE000ED00u);
inline auto* const NVIC = reinterpret_cast<NvicRegisters*>(0xE000E100u);
inline auto* const DWT = reinterpret_cast<DwtRegisters*>(0xE0001000u);
inline auto* const COREDEBUG = reinterpret_cast<CoreDebugRegisters*>(0xE000EDF0u);

// --- Reset and clock control (RM0433 section 8.7) ---------------------------
struct RccRegisters {
    reg32 CR;            // 0x00
    reg32 HSICFGR;       // 0x04
    reg32 CRRCR;         // 0x08
    reg32 CSICFGR;       // 0x0C
    reg32 CFGR;          // 0x10
    reg32 reserved0;     // 0x14
    reg32 D1CFGR;        // 0x18
    reg32 D2CFGR;        // 0x1C
    reg32 D3CFGR;        // 0x20
    reg32 reserved1;     // 0x24
    reg32 PLLCKSELR;     // 0x28
    reg32 PLLCFGR;       // 0x2C
    reg32 PLL1DIVR;      // 0x30
    reg32 PLL1FRACR;     // 0x34
    reg32 PLL2DIVR;      // 0x38
    reg32 PLL2FRACR;     // 0x3C
    reg32 PLL3DIVR;      // 0x40
    reg32 PLL3FRACR;     // 0x44
    reg32 reserved2;     // 0x48
    reg32 D1CCIPR;       // 0x4C
    reg32 D2CCIP1R;      // 0x50
    reg32 D2CCIP2R;      // 0x54
    reg32 D3CCIPR;       // 0x58
    reg32 reserved3;     // 0x5C
    reg32 CIER;          // 0x60
    reg32 CIFR;          // 0x64
    reg32 CICR;          // 0x68
    reg32 reserved4;     // 0x6C
    reg32 BDCR;          // 0x70
    reg32 CSR;           // 0x74
    reg32 reserved5;     // 0x78
    reg32 AHB3RSTR;      // 0x7C
    reg32 AHB1RSTR;      // 0x80
    reg32 AHB2RSTR;      // 0x84
    reg32 AHB4RSTR;      // 0x88
    reg32 APB3RSTR;      // 0x8C
    reg32 APB1LRSTR;     // 0x90
    reg32 APB1HRSTR;     // 0x94
    reg32 APB2RSTR;      // 0x98
    reg32 APB4RSTR;      // 0x9C
    reg32 GCR;           // 0xA0
    reg32 reserved6;     // 0xA4
    reg32 D3AMR;         // 0xA8
    reg32 reserved7[9];  // 0xAC..0xCC
    reg32 RSR;           // 0xD0
    reg32 AHB3ENR;       // 0xD4
    reg32 AHB1ENR;       // 0xD8
    reg32 AHB2ENR;       // 0xDC
    reg32 AHB4ENR;       // 0xE0
    reg32 APB3ENR;       // 0xE4
    reg32 APB1LENR;      // 0xE8
    reg32 APB1HENR;      // 0xEC
    reg32 APB2ENR;       // 0xF0
    reg32 APB4ENR;       // 0xF4
};
inline auto* const RCC = reinterpret_cast<RccRegisters*>(0x58024400u);

struct PwrRegisters {
    reg32 CR1, CSR1, CR2, CR3, CPUCR;
    reg32 reserved0;
    reg32 D3CR;
};
inline auto* const PWR = reinterpret_cast<PwrRegisters*>(0x58024800u);

struct FlashRegisters {
    reg32 ACR;
};
inline auto* const FLASH = reinterpret_cast<FlashRegisters*>(0x52002000u);

// --- GPIO -------------------------------------------------------------------
struct GpioRegisters {
    reg32 MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR;
    reg32 AFR[2];
};
inline auto* const GPIOA = reinterpret_cast<GpioRegisters*>(0x58020000u);
inline auto* const GPIOB = reinterpret_cast<GpioRegisters*>(0x58020400u);
inline auto* const GPIOC = reinterpret_cast<GpioRegisters*>(0x58020800u);
inline auto* const GPIOD = reinterpret_cast<GpioRegisters*>(0x58020C00u);
inline auto* const GPIOE = reinterpret_cast<GpioRegisters*>(0x58021000u);

// --- ADC (RM0433 section 25.7) ---------------------------------------------
struct AdcRegisters {
    reg32 ISR, IER, CR, CFGR, CFGR2, SMPR1, SMPR2, PCSEL;
    reg32 LTR1, HTR1;
    reg32 reserved0[2];
    reg32 SQR1, SQR2, SQR3, SQR4;
    reg32 DR;
};
struct AdcCommonRegisters {
    reg32 CSR;
    reg32 reserved0;
    reg32 CCR, CDR, CDR2;
};
inline auto* const ADC1 = reinterpret_cast<AdcRegisters*>(0x40022000u);
inline auto* const ADC12_COMMON = reinterpret_cast<AdcCommonRegisters*>(0x40022300u);

// --- Timers -----------------------------------------------------------------
struct TimRegisters {
    reg32 CR1, CR2, SMCR, DIER, SR, EGR, CCMR1, CCMR2, CCER, CNT, PSC, ARR, RCR;
    reg32 CCR[4];
    reg32 BDTR;
};
inline auto* const TIM1 = reinterpret_cast<TimRegisters*>(0x40010000u);
inline auto* const TIM2 = reinterpret_cast<TimRegisters*>(0x40000000u);
inline auto* const TIM3 = reinterpret_cast<TimRegisters*>(0x40000400u);

// --- I2C (RM0433 section 52.7) ---------------------------------------------
struct I2cRegisters {
    reg32 CR1, CR2, OAR1, OAR2, TIMINGR, TIMEOUTR, ISR, ICR, PECR, RXDR, TXDR;
};
inline auto* const I2C1 = reinterpret_cast<I2cRegisters*>(0x40005400u);

// --- USART ------------------------------------------------------------------
struct UsartRegisters {
    reg32 CR1, CR2, CR3, BRR, GTPR, RTOR, RQR, ISR, ICR, RDR, TDR, PRESC;
};
inline auto* const USART1 = reinterpret_cast<UsartRegisters*>(0x40011000u);

// --- DMA and DMAMUX ---------------------------------------------------------
struct DmaStreamRegisters {
    reg32 CR, NDTR, PAR, M0AR, M1AR, FCR;
};
struct DmaRegisters {
    reg32 LISR, HISR, LIFCR, HIFCR;
    DmaStreamRegisters stream[8];
};
struct DmamuxChannelRegisters {
    reg32 CCR;
};
inline auto* const DMA1 = reinterpret_cast<DmaRegisters*>(0x40020000u);
inline auto* const DMAMUX1 = reinterpret_cast<DmamuxChannelRegisters*>(0x40020800u);

// --- Interrupt numbers used here -------------------------------------------
enum Irq : int32_t {
    kDma1Stream0 = 11,
    kDma1Stream1 = 12,
    kTim2 = 28,
    kUsart1 = 37,
    kI2c1Event = 31,
};

inline void nvic_enable(Irq irq, uint8_t priority) {
    const uint32_t n = static_cast<uint32_t>(irq);
    NVIC->IPR[n] = static_cast<uint8_t>(priority << 4);  // four priority bits
    NVIC->ISER[n >> 5] = 1u << (n & 0x1Fu);
}

// Data and instruction synchronisation. Needed after any write that changes
// how a later access behaves: clock enables, MPU or cache configuration and
// the vector table.
inline void dsb() { asm volatile("dsb 0xF" ::: "memory"); }
inline void isb() { asm volatile("isb 0xF" ::: "memory"); }

}  // namespace pa::platform
