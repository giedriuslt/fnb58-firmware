/* Bootloader: the first 64 KiB of the AT32F403ACGT7's 1 MiB bank.
 * The application is linked at 0x08010000 (firmware/memory.x) and the
 * image-valid trailer occupies the last 2 KiB sector (0x080FF800, bank2)
 * — see bootloader/src/meta.rs. */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 64K
  RAM   : ORIGIN = 0x20000000, LENGTH = 96K
}
