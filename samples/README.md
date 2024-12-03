# Overview

This sample application provides an example usage of the touch screen TSC2003 from TI.
The example includes :

- Touch pressure measurement.
- Screen inversion and coordinate swapping.
- Debounce Mechanism.

# Requirements

- Hardware: Zest Display LCD shield with TSC2003-compatible touch screen.
- Any Zest Core board.
- Configuration Options:
- `CONFIG_INPUT=y` in prj.conf to use INPUT API.
- `CONFIG_I2C=y` in prj.conf to use I2C API.
- `CONFIG_INPUT_GPIO_KEYS=n` in prj.conf to disables GPIO keys.
- `CONFIG_TSC2003_DEBOUNCE_MS=200` in prj to set debounce time.

# References

- [TSC2003 Datasheet](https://www.ti.com/lit/ds/symlink/tsc2003.pdf?ts=1730931724485&ref_url=https%253A%252F%252Fwww.google.com%252F).

# Building and Running

```shell
cd <driver_directory>
west build -p always -b <BOARD> -- -D DTC_OVERLAY_FILE=sixtron_bus.overlay
west flash
```

# Sample Output

```shell
*** Booting Zephyr OS build v3.7.0 ***

Touch sample for touchscreen: TSC2003
TOUCH PRESS X, Y: (32, 48)
TOUCH RELEASE X, Y: (32, 48)

```
