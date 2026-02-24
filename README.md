# SHT30-D Temperature/Humidity Sensor with nRF52840 DK

Firmware for the **HKD GY - SHT30-D** (or any SHT30-D compatible) temperature/humidity sensor on the nRF52840 Development Kit. Uses the sensor's ALERT pin to wake the nRF52840 when temperature or humidity exceeds configured thresholds. **Advertises breach data over BLE** in CBOR format for power-efficient transmission.

## Wiring

| SHT30-D Pin | nRF52840 DK Pin | Arduino Header |
|-------------|-----------------|----------------|
| VDD         | 3V3             | 3V3 or VDD     |
| GND         | GND             | GND            |
| SDA         | P0.26           | A4 / D14 (SDA) |
| SCL         | P0.27           | A5 / D15 (SCL) |
| **ALERT**   | **P1.3**        | **D2**         |

### I2C Address

- **0x44** – ADDR pin connected to GND (default in overlay)
- **0x45** – ADDR pin connected to VDD

If your breakout uses 0x45, edit `boards/nrf52840dk_nrf52840.overlay` and change `reg = <0x45>`.

### Pull-ups

The nRF52840 DK provides I2C pull-ups on the Arduino header. External pull-ups are usually not needed.

## Build & Flash

```bash
west build -b nrf52840dk/nrf52840
west flash
```

Connect a serial terminal at 115200 baud to see output.

## Behavior

1. Sensor is initialized and thresholds are set (see `ALERT_TEMP_LO/HI` and `ALERT_HUMIDITY_LO/HI` in `src/main.c`).
2. The main thread sleeps until the ALERT pin asserts (threshold breach).
3. When temperature or humidity goes outside the configured range, the SHT30-D drives ALERT high.
4. The nRF52840 wakes, reads values, **advertises only the breached value(s) over BLE** in CBOR format, and polls until back to normal.
5. When values return within range, BLE advertising stops.

## BLE Advertising (CBOR)

On breach, manufacturer data is advertised:

- **Company ID**: 0x05F1 (change `COMPANY_ID` in `src/main.c` for your product).
- **CBOR payload**: map with `"t"` (temperature in °C) and/or `"h"` (humidity in %RH) — only breached values are included.
- **Example**: Temp breach only → `{"t": 36.5}`; both → `{"t": 36.5, "h": 85.2}`.
- Advertising stops when values return to normal.

Use a BLE scanner (e.g. nRF Connect app) and a CBOR decoder to read the data.
