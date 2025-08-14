
### Write programm to MCU? example:
```bash
st-flash write test2/.pio/build/genericSTM32F103C8/firmware.bin 0x8000000

st-flash 1.6.0
2025-08-14T13:08:39 INFO usb.c: -- exit_dfu_mode
2025-08-14T13:08:39 INFO common.c: Loading device parameters....
2025-08-14T13:08:39 INFO common.c: Device connected is: F1 Medium-density device, id 0x20036410
2025-08-14T13:08:39 INFO common.c: SRAM size: 0x5000 bytes (20 KiB), Flash: 0x10000 bytes (64 KiB) in pages of 1024 bytes
2025-08-14T13:08:39 INFO common.c: Attempting to write 960 (0x3c0) bytes to stm32 address: 134217728 (0x8000000)
Flash page at addr: 0x08000000 erased
2025-08-14T13:08:39 INFO common.c: Finished erasing 1 pages of 1024 (0x400) bytes
2025-08-14T13:08:39 INFO common.c: Starting Flash write for VL/F0/F3/F1_XL core id
2025-08-14T13:08:39 INFO flash_loader.c: Successfully loaded flash loader in sram
  1/1 pages written
2025-08-14T13:08:39 INFO common.c: Starting verification of write complete
2025-08-14T13:08:39 INFO common.c: Flash written and verified! jolly good!

```
