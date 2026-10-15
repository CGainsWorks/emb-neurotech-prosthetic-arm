#include "uart.hpp"

#include <cstring>

#include "pa/config/board.hpp"
#include "gpio.hpp"
#include "registers.hpp"
#include "system.hpp"

namespace pa::platform::uart {
namespace {

// Two hundred milliseconds of telemetry at the rates in board.hpp, so a brief
// stall on the host side does not cost frames.
constexpr std::size_t kTxRingBytes = 4096;
constexpr std::size_t kRxQueueDepth = 8;

__attribute__((section(".dma_buffer"), aligned(32)))
volatile uint8_t g_tx_ring[kTxRingBytes];

volatile std::size_t g_tx_head = 0;  // written by send()
volatile std::size_t g_tx_tail = 0;  // advanced when a DMA burst completes
volatile std::size_t g_tx_in_flight = 0;
volatile bool g_tx_busy = false;
volatile uint32_t g_dropped = 0;

protocol::FrameDecoder g_decoder;
protocol::DecodedFrame g_rx_queue[kRxQueueDepth];
volatile std::size_t g_rx_head = 0;
volatile std::size_t g_rx_tail = 0;

std::size_t ring_free() {
    const std::size_t head = g_tx_head;
    const std::size_t tail = g_tx_tail;
    return head >= tail ? kTxRingBytes - (head - tail) - 1 : tail - head - 1;
}

void start_transmit() {
    if (g_tx_busy) return;
    const std::size_t head = g_tx_head;
    const std::size_t tail = g_tx_tail;
    if (head == tail) return;

    // One contiguous run at a time. The DMA cannot wrap the ring for us, so a
    // burst stops at the end of the buffer and the next one picks up from the
    // start.
    const std::size_t run = head > tail ? head - tail : kTxRingBytes - tail;

    DmaStreamRegisters& stream = DMA1->stream[1];
    stream.CR &= ~1u;
    while (stream.CR & 1u) {
    }
    DMA1->LIFCR = 0x0F400000u;  // clear stream 1 flags
    stream.PAR = reinterpret_cast<uint32_t>(&USART1->TDR);
    stream.M0AR = reinterpret_cast<uint32_t>(&g_tx_ring[tail]);
    stream.NDTR = run;
    stream.CR = (1u << 6)    // memory to peripheral
              | (1u << 10)   // memory increment
              | (2u << 16)   // high priority
              | (1u << 4);   // transfer complete interrupt
    g_tx_in_flight = run;
    g_tx_busy = true;
    stream.CR |= 1u;
}

}  // namespace

bool begin() {
    RCC->AHB4ENR |= (1u << 0);   // GPIOA
    RCC->APB2ENR |= (1u << 4);   // USART1
    RCC->AHB1ENR |= (1u << 0);   // DMA1
    dsb();

    // PA9 transmit, PA10 receive, alternate function 7.
    gpio::configure(GPIOA, 9, gpio::Mode::kAlternate, gpio::Pull::kNone, gpio::Speed::kVeryHigh,
                    gpio::Type::kPushPull, 7);
    gpio::configure(GPIOA, 10, gpio::Mode::kAlternate, gpio::Pull::kUp, gpio::Speed::kVeryHigh,
                    gpio::Type::kPushPull, 7);

    USART1->CR1 = 0;
    // Oversampling by 16, so the divisor is the clock over the baud rate.
    USART1->BRR = kApbClockHz / config::kUartBaud;
    USART1->CR3 = (1u << 7);  // DMAT
    USART1->CR1 = (1u << 0)   // UE
                | (1u << 2)   // RE
                | (1u << 3)   // TE
                | (1u << 5);  // RXNEIE

    DMAMUX1[1].CCR = 42u;  // USART1 transmit request (RM0433 table 121)
    nvic_enable(kDma1Stream1, 2);
    nvic_enable(kUsart1, 2);
    return true;
}

bool send(const uint8_t* data, std::size_t length) {
    if (length == 0) return true;
    if (length > ring_free()) {
        ++g_dropped;
        return false;
    }
    std::size_t head = g_tx_head;
    for (std::size_t i = 0; i < length; ++i) {
        g_tx_ring[head] = data[i];
        head = (head + 1) % kTxRingBytes;
    }
    g_tx_head = head;
    start_transmit();
    return true;
}

uint32_t dropped() { return g_dropped; }

bool poll(protocol::DecodedFrame& frame) {
    if (g_rx_tail == g_rx_head) return false;
    frame = g_rx_queue[g_rx_tail];
    g_rx_tail = (g_rx_tail + 1) % kRxQueueDepth;
    return true;
}

uint32_t link_errors() {
    return g_decoder.crc_errors() + g_decoder.malformed() + g_decoder.overflows();
}

extern "C" void DMA1_Stream1_IRQHandler() {
    if (DMA1->LISR & (1u << 11)) {  // transfer complete on stream 1
        DMA1->LIFCR = (1u << 11);
        g_tx_tail = (g_tx_tail + g_tx_in_flight) % kTxRingBytes;
        g_tx_in_flight = 0;
        g_tx_busy = false;
        start_transmit();
    }
}

extern "C" void USART1_IRQHandler() {
    // Overrun, framing, noise and parity all leave a flag that has to be
    // cleared or the receiver stops. Clearing them and carrying on is right:
    // whatever byte was involved will fail its frame's CRC.
    const uint32_t status = USART1->ISR;
    if (status & 0x0Fu) USART1->ICR = 0x0Fu;

    if (status & (1u << 5)) {  // RXNE
        const uint8_t byte = static_cast<uint8_t>(USART1->RDR);
        protocol::DecodedFrame frame;
        if (g_decoder.push(byte, frame) == protocol::DecodeResult::kFrame) {
            const std::size_t next = (g_rx_head + 1) % kRxQueueDepth;
            // A full queue means the loop has stopped reading commands, which
            // is a fault in its own right. Dropping the newest keeps the
            // oldest and the command timeout will fire either way.
            if (next != g_rx_tail) {
                g_rx_queue[g_rx_head] = frame;
                g_rx_head = next;
            }
        }
    }
}

}  // namespace pa::platform::uart
