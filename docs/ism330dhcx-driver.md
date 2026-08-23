# ISM330DHCX Thin Driver

## Purpose

This driver provides a small, application-specific interface for the ISM330DHCX accelerometer and gyroscope used on the X-NUCLEO-IKS02A1 expansion board.

## Scope

### Supported

- I2C communication
- Device identification
- Software reset
- Block-data update
- Register auto-increment
- Accelerometer and gyroscope ODR configuration
- Full-scale configuration
- Performance-mode configuration
- Raw six-axis sample reading
- Conversion to physical units
- Timeout and error reporting

### Not currently supported

- SPI
- FIFO
- Data-ready interrupts
- Embedded finite-state machine
- Machine Learning Core
- Sensor hub
- Self-test
- Calibration

## Hardware and Transport

| Property | Value |
|---|---|
| MCU board | NUCLEO-L476RG |
| Sensor board | X-NUCLEO-IKS02A1 |
| Sensor | ISM330DHCX |
| Transport | I2C1 |
| Project 7-bit address | `0x6B` |
| STM32 HAL address | `0xD6` |
| Transaction model | Blocking with finite timeout |

The STM32 HAL expects the 7-bit address shifted left by one.

## Device Identification

| Property | Value |
|---|---|
| `WHO_AM_I` register | `0x0F` |
| Expected value | `0x6B` |

The I2C address and `WHO_AM_I` value are both `0x6B`, but they represent different properties.


## Initialization Sequence

1. Initialize the device object.
2. Read and validate `WHO_AM_I`.
3. Request a software reset.
4. Poll until reset completes or times out.
5. Configure BDU and register auto-increment.
6. Configure ODR, range and performance modes.
7. Read and convert samples.

## Important Registers

| Register | Address | Fields used |
|---|---:|---|
| `WHO_AM_I` | `0x0F` | Device identification |
| `CTRL1_XL` | `0x10` | Accelerometer ODR and range |
| `CTRL2_G` | `0x11` | Gyroscope ODR and range |
| `CTRL3_C` | `0x12` | Reset, BDU and auto-increment |
| `CTRL6_C` | `0x15` | Accelerometer performance mode |
| `CTRL7_G` | `0x16` | Gyroscope performance mode |
| `OUTX_L_G` | `0x22` | First byte of the 12-byte sample |

## ODR and Mode Restrictions

Document the relationships from the datasheet.

| Accelerometer ODR | Low-power/normal | High-performance |
|---|---|---|
| 1.6 Hz | Valid | Invalid |
| 12.5–52 Hz | Valid, low-power | Valid |
| 104–208 Hz | Valid, normal | Valid |
| 416 Hz and higher | Invalid | Valid |

## Scale Factors

### Accelerometer

| Range | Sensitivity |
|---|---:|
| ±2 g | `0.000061 g/LSB` |
| ±4 g | `0.000122 g/LSB` |
| ±8 g | `0.000244 g/LSB` |
| ±16 g | `0.000488 g/LSB` |

### Gyroscope

| Range | Sensitivity |
|---|---:|
| ±125 dps | `0.004375 dps/LSB` |
| ±250 dps | `0.008750 dps/LSB` |
| ±500 dps | `0.017500 dps/LSB` |
| ±1000 dps | `0.035000 dps/LSB` |
| ±2000 dps | `0.070000 dps/LSB` |
| ±4000 dps | `0.140000 dps/LSB` |

## Data Conversion

Acceleration:

`raw LSB × sensitivity g/LSB × 9.80665 m/s²/g`

Angular rate:

`raw LSB × sensitivity dps/LSB`

## Assumptions and Limitations

- Sensor data is little-endian.
- Each raw axis is signed 16-bit two's-complement.
- BDU and auto-increment are enabled.
- The current application polls every 500 ms.
- The sensor ODR is 104 Hz, so intermediate samples are skipped.
- The driver is not currently thread-safe.
- Configuration state is valid only when `sensor_configured` is true.
- A reset invalidates the stored configuration.

## Verification

The driver is verified on a NUCLEO-L476RG with an X-NUCLEO-IKS02A1 expansion board.

- Correct `WHO_AM_I`
- Reset completes before timeout
- Stationary gyroscope is close to zero
- One accelerometer axis measures approximately ±9.81 m/s²
- Values respond correctly when the board is rotated

## References
- ST ISM330DHCX component driver
- ISM330DHCX datasheet