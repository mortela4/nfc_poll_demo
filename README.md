# nfc_poll_demo

NFC polling demo for PlatformIO/Arduino w. ESP32-S3 Mini-M1 kit and "ST25R3911B-DISCO" kit.

NOTE: configured to read NFC-A tags (ISO14443(A)).



# Setup

"config.h" contains the GPIO definitions for the SPI.
Set to work w. ESP32-S3 Mini-M1 kit connected to SPI-header on ST25R3911B-DISCO kit.
Can also work with the X-NUCLEO-NFC05A1 add-on kit.

The RFAL-lib (from STmicro) can be configured in the "src/rfal_platform/rfal_platform.h" file.

# ST RFAL implementation

- src/rfal_core
- src/rfal_core/st25r3911
- src/rfal_platform


# Debug Output:

Each step returns the NFC lib error code.
The uid represent the unique id of the nfc device.

```text
init
0
Worker
init poller
0
field on and start GT
0
Inventorying... 
0
uid: 9186832468014224
```


