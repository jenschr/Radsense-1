# Radsense1

Open source Radsense1 firmware for use with the Arduino IDE. Also available as [a PlatformIO project](https://github.com/jenschr/Radsense1). Get the [hardware from Maketronics](https://maketronics.no/radsense-1/). Learn more about the device and [how to program it with PlatformIO](https://maketronics.no/support/radsense-1-with-platformio/) on [the Maketronics support pages](https://maketronics.no/support/).

## Release v1.72

- Solves a bug that only applies to the Arduino IDE by setting pinMode for the RX/TX pins used to talk to the radar. It looks like Arduino-esp32 no longer does this by default, so we must ensure that it's set.

## Release v1.71

- Fix stuck mode where target walks off to the side
- Update BLE to support the latest NimBLE version (2.3.6). Device will now output 3 different characteristics: Data (what the radar sees), Settings (current modes on the device), Debug (some data that might be useful in rare cases)
- Ensure that BLE always turns on when the user button is pushed
- Fix rare case where the radar does not initialize properly by blinking error message and then letting Watchdog boot the device to try again