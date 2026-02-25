# SHT30-D Temperature/Humidity Sensor with nRF52840 DK

Firmware for the **HKD GY - SHT30-D** (or any SHT30-D compatible) temperature/humidity sensor on the nRF52840 Development Kit. Uses the sensor's ALERT pin to wake on threshold breach, **Button 1** for manual readings, and **BLE GATT** for events log retrieval.

## Wiring

| SHT30-D Pin | nRF52840 DK Pin | Arduino Header |
|-------------|-----------------|----------------|
| VDD         | 3V3             | 3V3 or VDD     |
| GND         | GND             | GND            |
| SDA         | P0.26           | A4 / D14 (SDA) |
| SCL         | P0.27           | A5 / D15 (SCL) |
| **ALERT**   | **P1.3**        | **D2**         |

**Button 1 (SW1)** on the nRF52840 DK is used for manual on-demand readings. No extra wiring required.

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

---

## How It Works – Functional Flow

### 1. Idle Mode (Normal Operation)

- Device sleeps, waiting for either a **threshold breach** (ALERT pin) or **Button 1** press.
- **No BLE advertising** when idle (power-efficient). To connect and fetch logs, press **Button 1** first.

### 2. Threshold Breach (ALERT Pin)

When temperature or humidity goes outside the configured range (see `ALERT_TEMP_LO/HI`, `ALERT_HUMIDITY_LO/HI` in `src/main.c`):

1. SHT30-D drives ALERT high → MCU wakes.
2. Sensor reads current T/H.
3. Logs breach lines to `events.log` (format: `{recycle}|{uptime}A|T{val}` or `H{val}`).
4. Switches to **non-connectable BLE advertising** with CBOR payload (only breached values).
5. Polls sensor every 2 seconds until values return to normal.
6. Logs back-to-normal lines (`...N|...`).
7. Stops breach advertising and returns to **idle (connectable)** advertising.

### 3. Button 1 – Manual Reading

When **Button 1 (SW1)** is pressed:

1. MCU wakes from idle.
2. Reads current temperature and humidity.
3. Logs to `events.log` with format `{recycle}|{uptime}M|T{val}` and `...M|H{val}` (M = manual).
4. Switches to **connectable BLE advertising** with CBOR payload containing both T and H.
5. User can connect and fetch `events.log` via GATT (see below).
6. After 60 seconds, returns to idle connectable advertising (without T/H in manufacturer data).

### 4. Logging Into the Device via BLE (Events Log Retrieval)

To fetch `events.log` over BLE, you must first **press Button 1** to start connectable advertising (device advertises for 60 seconds with T/H in manufacturer data). Then:

1. **Connect** to the device (name `TempHum`).
2. Discover the custom GATT service (128‑bit UUID).
3. **Write** ASCII `"00"` (2 bytes: `0x30 0x30`) to the **command characteristic**.
4. **Read** the **events characteristic** to retrieve `events.log` content.
5. Use long read (blob read) with offset to fetch the full file if it exceeds one ATT_MTU.

---

## Events Log Format

`events.log` is stored in LittleFS at `/lfs1/events.log` (external QSPI flash). Each line:

```
{recycle}|{uptime}{type}|{T|H}{value}
```

- **recycle**: Power-cycle counter (persisted).
- **uptime**: Seconds since boot.
- **type**: `A` = breach alert, `N` = back to normal, `M` = manual (Button 1).
- **T** = temperature (°C), **H** = humidity (%RH).

Example:

```
2|120A|T36
2|120A|H85
2|300N|T24
2|300N|H50
3|450M|T23
3|450M|H48
```

---

## BLE Advertising Summary

| State             | Mode               | T/H in advertisement?   |
|-------------------|--------------------|-------------------------|
| Idle              | None               | —                       |
| Threshold breach  | Non-connectable   | Yes (breached only)     |
| Button 1 manual   | Connectable       | Yes (both T and H)      |

**CBOR payload** (when T/H are advertised):

- `{"t": 36.5}` – temperature only
- `{"h": 85.2}` – humidity only
- `{"t": 36.5, "h": 85.2}` – both (manual or both breached)

Use a BLE scanner (e.g. nRF Connect app) and a CBOR decoder to read manufacturer data.
