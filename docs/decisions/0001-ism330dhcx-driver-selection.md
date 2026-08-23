# ADR-0001: Use a Thin Custom ISM330DHCX Driver

- Status: Accepted
- Date: 2026-08-23

## Context

This project requires basic ISM330DHCX configuration and six-axis sample acquisition on an STM32L476RG.

The project also has a learning objective: understand register-level sensor communication, configuration validation, raw-data decoding and physical-unit conversion.

Two approaches were evaluated:

1. Develop a small application-specific driver.
2. Integrate ST's official platform-independent component driver.

## Decision Criteria

- Readability
- Required feature coverage
- Portability
- Error reporting
- Testability
- Maintenance cost
- Register-level learning value
- Code size and complexity

## Comparison

| Area | Thin custom driver | ST component driver |
|---|---|---|
| Platform integration | Direct STM32 HAL I2C dependency | Platform read/write callbacks |
| Transport support | I2C only | I2C and SPI |
| Register coverage | Required subset | Comprehensive |
| Configuration API | Composite configuration | Individual setters |
| Device state | Stores active configuration | Primarily transport context |
| Error reporting | Project-specific status values | Zero/nonzero return values |
| Data output | Raw and converted sample structures | Raw getters and conversion helpers |
| Portability | STM32-specific | Platform-independent |
| Complexity | Small | Significantly larger |
| Learning value | Exposes design decisions | Demonstrates production-scale coverage |

## Decision

Use the thin custom driver for the current project.

The custom driver keeps the implementation understandable while supporting the functionality required by the current milestone.

ST's driver remains the reference implementation for checking register values, scale factors, configuration ordering and unsupported features.

## Consequences

### Benefits

- Smaller public API
- Clear project-specific status values
- Explicit configuration state
- Easier to explain in the portfolio
- Direct understanding of register operations

### Trade-offs

- STM32 HAL dependency
- I2C-only implementation
- Smaller feature set
- More responsibility for datasheet validation
- Custom code requires custom testing

## Revisit Conditions

Reconsider using ST's driver if the project requires:

- SPI support
- FIFO
- Interrupt routing
- Sensor hub
- Embedded FSM
- Machine Learning Core
- Multiple MCU platforms
- Complete production qualification

## References

- ST ISM330DHCX component driver
- ISM330DHCX datasheet