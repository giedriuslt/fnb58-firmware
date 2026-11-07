/* AT32F403ACGT7: 1 MiB flash at 0x08000000, 96 KiB base SRAM at 0x20000000.
 * The 128 KiB SRAM extension (option bytes) is not assumed.
 *
 * r180 layout under the bootloader (bootloader/, first 64 KiB of the
 * bank): this image is linked at 0x08010000 and main() must point VTOR
 * there before any interrupt can fire. The last 2 KiB sector of the bank
 * (0x080FF800, bank2) is the image-valid trailer the bootloader
 * maintains and COMMITs — excluded from this region.
 */
MEMORY
{
  FLASH : ORIGIN = 0x08010000, LENGTH = 958K
  RAM   : ORIGIN = 0x20000000, LENGTH = 96K
}

/* The framebuffer-free scaffold places nothing special. */
