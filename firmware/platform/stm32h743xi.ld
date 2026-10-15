/* Linker script for the STM32H743VIT6: 2 MB flash, 128 KB DTCM, 512 KB AXI SRAM.
 *
 * Two things here are decisions rather than boilerplate.
 *
 * The stack and the ordinary data live in DTCM, which is on its own bus port
 * and never contends with the AXI bus the DMA controllers use. That is what
 * keeps the interrupt latency deterministic while eight ADC channels and a
 * UART are streaming.
 *
 * The DMA buffers cannot live there. DTCM is reachable only by the core, so a
 * DMA controller told to write into it fails silently, which is the single
 * most common way an STM32H7 project loses its first week. They get their own
 * section in AXI SRAM instead and startup.cpp zeroes it explicitly.
 *
 * This has never been flashed.
 */

ENTRY(Reset_Handler)

MEMORY
{
  FLASH   (rx)  : ORIGIN = 0x08000000, LENGTH = 2048K
  DTCM    (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
  AXISRAM (rwx) : ORIGIN = 0x24000000, LENGTH = 512K
}

/* 8 KB of stack. The deepest call chain is the control task, which is a few
 * hundred bytes of locals plus the inverse dynamics working set; the rest is
 * margin for the three interrupt handlers, none of which nests.
 */
_stack_size = 8K;
_estack = ORIGIN(DTCM) + LENGTH(DTCM);
/* The same address, under a name the vector table can refer to as a constant
 * expression. startup.cpp declares it as a function for that reason; it is not
 * one and is never called.
 */
PROVIDE(_estack_symbol = _estack);

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(512);           /* the vector table alignment the core requires */
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } > FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    KEEP(*(.init))
    KEEP(*(.fini))
    . = ALIGN(4);
    _etext = .;
  } > FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } > FLASH

  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > FLASH
  .ARM :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } > FLASH

  .preinit_array :
  {
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
  } > FLASH

  .init_array :
  {
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
  } > FLASH

  .fini_array :
  {
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
  } > FLASH

  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } > DTCM AT> FLASH

  .bss :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } > DTCM

  /* newlib's sbrk wants a heap start symbol even in a build that never
   * allocates. Nothing here calls malloc and --specs=nosys.specs pulls the
   * stub in regardless, so the symbol is provided and the heap is empty.
   */
  PROVIDE(end = _ebss);
  PROVIDE(_end = _ebss);
  PROVIDE(__end__ = _ebss);
  PROVIDE(__HeapBase = _ebss);
  PROVIDE(__HeapLimit = _ebss);

  /* Nothing is placed after this in DTCM, so the check is that the stack has
   * its full depth below the top of the region.
   */
  ._stack_guard :
  {
    . = ALIGN(8);
    . = . + _stack_size;
    . = ALIGN(8);
  } > DTCM

  /* Reachable by the DMA controllers. NOLOAD because startup.cpp zeroes it
   * rather than the loader filling it, which keeps the image the size of the
   * code instead of the size of the buffers.
   */
  .dma_buffer (NOLOAD) :
  {
    . = ALIGN(32);
    _sdma = .;
    *(.dma_buffer)
    *(.dma_buffer*)
    . = ALIGN(32);
    _edma = .;
  } > AXISRAM

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
