# Texas Instruments tsc2003 touch screen Driver

Texas Instruments tsc2003 touch screen driver for Zephyr OS.

## Hardware requirements
- Zest_Display_LCD
- Any Zest Core board

## Usage
This touch screen driver can be used for basic touch functionalities, including X/Y position, pressure measurements, and sending touch events.

### Build
Use the Zest_Display_LCD shield:
```bash
west build -b <board> samples -- -DSHIELD=zest_display_lcd
```

Build the example:
```bash
west build -b <board> samples
```
