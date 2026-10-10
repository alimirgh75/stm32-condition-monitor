# STM32 checkpoint verification

## Development observations

The NUCLEO-L476RG / X-NUCLEO-IKS02A1 development sessions demonstrated DRDY,
DMA completion, timestamped samples, accelerometer RMS/peak responses to
movement, severity decay/hysteresis, on-demand status, impact-reference
commands and repeatable start/stop commands. Short diagnostics produced zero
sensor drops in normal measurements. Recorded DRDY intervals were approximately
8883-8889 us. These observations predate the cleanup changes below.

## Cleanup checks, 2026-10-10

```sh
make -C tests/host test
```

All 11 host checks pass. They also passed with AddressSanitizer and
UndefinedBehaviorSanitizer enabled (leak detection disabled for the execution
environment).

The changed C files also passed a host C syntax check using the bundled
STM32/CMSIS/FreeRTOS headers and a temporary Newlib type declaration. This
is not an ARM compile or link. A separate source-derived harness exercised
the UART callbacks/recovery helper with HAL/queue mocks: error recovery,
queue overflow, failed rearm retry, failed abort retry and unrelated UARTs
all passed. That harness is a development check, outside the committed suite.

| Check | Verified behavior |
|---|---|
| Monitor initialization | NORMAL and score zero |
| Warning entry/recovery | Known contribution enters WARNING; decay crosses its exit threshold |
| Completion during DMA start | Data is consumed and another read can start |
| Failed DMA start | Busy clears and the DRDY event remains pending for retry |
| DRDY arriving after snapshot | Previous timestamp remains correct; later event stays pending |
| Reads disabled during transfer | Completion drains while new DMA starts stay disabled |
| Multiple pending DRDY events | Newest event is consumed; older events increment drops |
| Constant window | Zero centered features, peak index zero and first timestamp |
| Known impulse | Expected RMS, absolute peak, magnitude and timestamp |
| Interrupted CLI line | Suffix is discarded through CR/LF; next line parses |
| Editing and length limit | Backspace works; 32 characters are accepted and 33 discarded |

The tests compile production sources with simulated peripheral outcomes.
They validate software logic, not electrical behavior, real DMA timing, UART
peripheral recovery or RTOS scheduling.

UART recovery has a task-side receive abort/rearm path. Callbacks request
recovery on HAL errors, failed rearm or a full RX queue. The interface clears
queued bytes and discards the interrupted line through the next CR/LF.
UART receive/error/drop counters are visible in `get status`. Persistent
recovery failures retry once per interface loop, with its existing one-tick
receive wait, and print one failure message until recovery succeeds.

## Board boundary

The cleanup was prepared without access to the board or an ARM build toolchain.
The modified firmware needs a CubeIDE build and flash before its board behavior
can be claimed as verified. A short check is:

1. Request `help` and `get status` after a complete window.
2. Apply `set impact-reference 2`, then confirm `get impact-reference`.
3. Issue `stop` twice; confirm STOPPED and the last measurement retained.
4. Issue `start` twice; confirm fresh windows accumulate.
5. Interrupt UART reception or overflow its RX queue. Press Enter after the
   recovery message, then confirm `help` works and the damaged line was not
   applied as a command.

Post-cleanup hardware verification remains outstanding.
