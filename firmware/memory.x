/* AT32F403ACGT7: 1 MiB flash at 0x08000000, 96 KiB base SRAM at 0x20000000.
 * The 128 KiB SRAM extension (option bytes) is not assumed. The stock
 * FNIRSI bootloader's flash layout is unknown until SWD access; this
 * scaffold takes over the whole bank.
 */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 1024K
  RAM   : ORIGIN = 0x20000000, LENGTH = 96K
}

/* The framebuffer-free scaffold places nothing special. */
