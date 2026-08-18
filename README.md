# FlowCAN

FlowCAN RTOS is a C11 FreeRTOS firmware for a combined PMW3901 optical-flow and VL53L1X time-of-flight module built around the **AT32F415KBU7-4**. It publishes sensors over native MSP V2 and DroneCAN v0 without a bootloader, dynamic allocation, or debug text on the MSP UART. This branch is the production-quality RTOS counterpart to the bare-metal `main` branch.

> V1 is source-, host-test-, and cross-build validated only. No physical board was available during implementation. SWD boot, electrical timing, sensor readings, LEDs, CAN recovery, flight-controller behavior, and endurance are **not hardware-validated**; use the [bring-up checklist](docs/bringup-checklist.md).

## Hardware and fixed clocks

- AT32F415KBU7-4: 128 KiB Flash, 32 KiB SRAM.
- 8 MHz HSE → 144 MHz SYSCLK; APB1/APB2 72 MHz (below the 75 MHz limit).
- FreeRTOS owns the 1 kHz SysTick; TMR2 remains a 1 MHz 32-bit free-running microsecond timer.
- SPI1 mode 3 at 1.125 MHz; I2C1 400 kHz; USART1 115200 8N1.
- CAN1 1 Mbit/s: prescaler 4, 18 TQ, BS1=13, BS2=4, sample point 77.8%.
- TMR1 CH1 + DMA drives one WS2812 at 800 kHz and is not shared with the timebase.

### Pinout

| Function | Pin | Notes |
|---|---:|---|
| PMW3901 CS / SCK / MISO / MOSI | PA4 / PA5 / PA6 / PA7 | SPI1 mode 3 |
| PMW3901 reset / motion | PB0 / PB1 | Motion falling-edge EXTI; ISR only queues work |
| VL53L1X XSHUT / GPIO1 | PB5 / PB8 | GPIO1 falling-edge EXTI |
| VL53L1X SCL / SDA | PB6 / PB7 | I2C1, 400 kHz; external pull-ups required |
| MSP TX / RX | PA9 / PA10 | USART1, 115200 8N1 |
| CAN RX / TX | PA11 / PA12 | External CAN transceiver required |
| WS2812 | PA8 | TMR1 CH1 DMA |
| Activity LEDs | PA0–PA3 | PMW, ToF, MSP RX, CAN RX; active low |
| SWDIO / SWCLK | PA13 / PA14 | SWD |

The design assumes the CAN transceiver has no separate EN/STBY control pin.

## Build

The default toolchain is the local Arm GNU **10.2.1 (2020-q4-major)** installation. Override `TOOLCHAIN` with a directory containing `arm-none-eabi-gcc`, `objcopy`, `size`, `readelf`, `objdump`, and `nm`.

```sh
make test       # native protocol/math/scheduler tests
make all        # ELF, HEX, BIN, MAP and size/entry/vector/static-allocation checks
make verify     # host tests + cross-build + repository policy checks
make release    # clean verification plus release directory and SHA-256 manifest
make clean
```

Outputs are `flowcan-rtos.elf`, `.hex`, `.bin`, and `.map` under `build/`; release files are under `build/release/`. The linker starts at `0x08000000`; no AM32 bootloader or EEPROM region is reserved. Automated limits fail above 128 KiB Flash or 32 KiB SRAM, verify the FreeRTOS exception vectors and static stack budget, and reject allocation functions.

## RTOS architecture

FreeRTOS-Kernel V11.3.0 uses its GCC ARM_CM3 port because AT32F415 has no FPU. Preemption and time slicing are enabled at a 1 kHz tick; tickless idle and software timers are disabled. Every task, queue, event group, protocol arena, and hardware ring is statically allocated, and no FreeRTOS heap implementation is linked.

| Task | Priority | Static stack | Responsibility |
|---|---:|---:|---|
| Communication | 4 | 512 words / 2048 B | Sole owner of libcanard, MSP parsing, UART/CAN publication and Bus-Off recovery |
| Optical Flow | 3 | 320 words / 1280 B | PMW3901 initialization, motion notification, accumulation and 50 Hz snapshots |
| Range | 2 | 384 words / 1536 B | VL53L1X state machine, data-ready notification, retry and 40 Hz snapshots |
| System | 1 | 256 words / 1024 B | Health, activity/status LEDs, stack monitoring and watchdog supervision |
| Idle | 0 | 128 words / 512 B | FreeRTOS idle task |

Flow and range samples cross length-one overwrite queues, so producers never block and only the newest sample is retained under congestion. PMW, ToF, UART and CAN ISRs wake tasks with direct notifications. CAN runs at NVIC priority 5, EXTI at 6, USART at 7 and WS2812 DMA at 8; SysTick and PendSV use the kernel's lowest priority.

### SWD flashing

Install Artery's OpenOCD support and pass its target script explicitly:

```sh
make flash AT32_OPENOCD_TARGET=/absolute/path/to/target/at32f415xx.cfg
```

The target intentionally fails if that script is absent. Stock OpenOCD installations commonly do not ship AT32F415 support. A successful build is not proof that a board was flashed.

## Configuration and calibration

Public configuration is centralized in [`config/config.h`](config/config.h): clocks, buses, node ID, rates, sensor timing, axis transforms, range limits/offset, retry periods, LEDs, watchdog, features, and firmware version.

`FLOW_RAD_PER_COUNT` defaults to **0.0015 rad/count and is provisional**. It is not a final PMW3901 scale. Verify rotation, axis signs, lens/height effects, then fit the factor from measured displacement before flight. The initial range values (`80…3500 mm`, zero offset) likewise require the final enclosure and optical window.

## MSP V2

USART1 continuously pushes native `$X<` frames:

| ID | Payload | Rate | Invalid behavior |
|---:|---|---:|---|
| `0x1F02` optical flow | quality `u8`, accumulated X `i32 LE`, accumulated Y `i32 LE` | 50 Hz | quality 0 and zero counts |
| `0x1F01` rangefinder | quality `u8`, distance mm `i32 LE` | 40 Hz | quality 0 and distance `-1` |

CRC is CRC-8/DVB-S2 over flags, command, length, and payload. The RX path has a fixed ring buffer and complete MSP V2 parser for validation/activity/future commands. V1 defines no private request protocol and does not impersonate a flight controller.

For INAV, connect the module UART to a serial port configured for MSP sensor input and verify axes and scale on the bench. For ArduPilot, the MSP path is the promised route for complete optical-flow fusion in this release.

## DroneCAN v0

- Static node ID `125`, name `com.flyingrc.flowcan`.
- `uavcan.protocol.NodeStatus`, `uavcan.protocol.GetNodeInfo`.
- `com.hex.equipment.flow.Measurement` and `uavcan.equipment.range_sensor.Measurement`.
- 2 KiB libcanard arena and fixed 16-frame hardware RX queue; no heap.
- Status/GetNodeInfo use high priority. Periodic sensor messages are low priority and are not queued while Bus-Off is active.
- Bus-Off sets the health flag, discards queued transmissions, waits one second, and reinitializes CAN.

The flow message contains radians integrated over the real sample interval. Because the board has no gyro, both `rate_gyro_integral` fields are IEEE-754 `NaN`, meaning unavailable. PX4 handles non-finite gyro integrals as unavailable. The current ArduPilot HereFlow DroneCAN backend does not make the same check, so this firmware does **not** claim ArduPilot DroneCAN optical-flow fusion; use MSP for that. Node discovery, decoding, and DroneCAN range remain in scope.

Range timestamps are zero because the node does not implement network time synchronization. The beam orientation is defined as downward in the body frame; valid/too-close/too-far/undefined map to the standard `reading_type` values.

The 16-bit NodeStatus vendor code exposes:

| Bit | Meaning |
|---:|---|
| 0 | PMW3901 fault |
| 1 | VL53L1X fault |
| 2 | I2C fault |
| 3 | CAN Bus-Off |
| 4 | CAN RX queue overflow |
| 5 | CAN/libcanard TX queue pressure |
| 6 | scheduler late |
| 7 | watchdog feed withheld |
| 8 | RTOS task stack low-water warning |
| 9 | fatal RTOS assertion/stack overflow |
| 10 | MSP UART TX ring pressure |

## Sensors and reliability

PMW3901 initialization verifies Product ID `0x49` and inverse ID `0xB6`, then applies the pinned Bitcraze MIT register sequence. Motion EXTI only notifies the optical-flow task; Motion Burst is read and accumulated there before publication clears the accumulator.

VL53L1X uses the ST ULD core in long continuous mode with a 20 ms timing budget and 25 ms inter-measurement period (40 Hz target). Every I2C phase has a 5 ms deadline. A failed transaction invalidates distance data, shuts the device down, and starts a staged one-second retry; stale distance is never relabeled as valid.

Activity LEDs stay on for 30 ms per event. WS2812 states are blue/startup, green/healthy, red/PMW fault, yellow/ToF fault, fast red/both sensors or fatal RTOS state, and purple/CAN Bus-Off. Each critical task reports a heartbeat; the System task refreshes the 500 ms independent watchdog only when Communication, Optical Flow and Range have all reported within the 100 ms supervision window. Stack overflow or an RTOS assertion deliberately stops feeding the watchdog.

## Repository layout

- `app/`: static task graph, supervision, scheduling helpers and status/activity indication.
- `platform/`: AT32 clocks/peripherals/interrupts and the ST ULD I2C binding.
- `drivers/`: PMW3901 and VL53L1X state machines.
- `protocol/`: MSP V2 codec/parser and DroneCAN transport/messages.
- `third_party/`: pinned FreeRTOS kernel, BSP, ST ULD, libcanard, DSDL and generated bindings.
- `tests/`: native deterministic tests.
- `tools/`: image limits, vector/entry/static-allocation and tree checks.

See [`THIRD_PARTY.md`](THIRD_PARTY.md) for versions and licenses.

## Validation boundary

Automated checks cover MSP golden layout/CRC/parser failures, DroneCAN encode/decode including NaN gyro and range/status mapping, PMW transform/accumulation/reset, 32-bit timer wrap scheduling, UART whole-frame capacity, heartbeat decisions and stack low-water decisions. They cross-build ELF/HEX/BIN/MAP, assert memory limits/vector base/FreeRTOS exception handlers/static stack budget/entry point, and search for allocation calls.

They do **not** validate task switching on AT32F415, ISR wakeups, measured stack margins, watchdog reset behavior, oscillator accuracy, electrical pin mapping, sensor communication, DMA waveform, CAN transceiver behavior, flight-controller setup, recovery on a physical bus, or 30-minute stability. Complete and retain evidence from [`docs/bringup-checklist.md`](docs/bringup-checklist.md) before calling the hardware flight-ready.
