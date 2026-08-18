# FlowCAN V1 hardware bring-up and evidence checklist

None of the items below are satisfied by a successful host test or firmware build. Record the board revision, firmware SHA-256, instrument, setup, result, and attached capture for every row.

| Area | Procedure | Acceptance evidence | Status |
|---|---|---|---|
| Power / SWD | Current-limited 3.3 V supply; connect SWDIO PA13, SWCLK PA14, GND; program and reset | Current draw, successful verify, PC at reset/main | Not tested |
| Clocks | Probe MCO/test build or timer-derived output | HSE 8 MHz, SYSCLK 144 MHz, APB1/APB2 72 MHz | Not tested |
| PMW3901 SPI | Capture PA5/6/7 and PA4 CS during ID read | Mode 3, ~1.125 MHz, ID `49/B6` | Not tested |
| PMW3901 motion | Move over textured target at several heights | Signed X/Y, quality, no burst corruption, selected rotation | Not tested |
| Flow scale | Traverse a measured path at fixed height and compare angular displacement | Replace provisional `FLOW_RAD_PER_COUNT=0.0015` with fitted value and uncertainty | Not tested |
| VL53L1X | Test 80 mm, 0.5 m, 1 m, 3.5 m and out-of-range targets | 400 kHz I2C, 20/25 ms timing, status mapping, offset | Not tested |
| Sensor recovery | Disconnect/reconnect SPI and I2C sensors | Invalid data is published; no stale distance; retry every ~1 s | Not tested |
| MSP | Logic-analyzer capture on PA9 at 115200 8N1 | Valid `$X<` frames, IDs `1F02/1F01`, CRC and cadence | Not tested |
| INAV | Configure MSP sensor input and inspect optical flow/range telemetry | Correct axes/counts/range and quality | Not tested |
| ArduPilot | Use the MSP sensor path for complete flow fusion | Flow and range accepted; EKF behavior checked safely | Not tested |
| DroneCAN | Connect through a proper CAN transceiver and DroneCAN GUI | Node 125, node name, NodeStatus, GetNodeInfo, flow/range decoding; range orientation remains undefined until body-down encoding is proven | Not tested |
| PX4 | Inspect DroneCAN optical flow and distance sensor topics | NaN gyro is treated unavailable; flow/range fields correct | Not tested |
| Bus-Off | Force CAN dominant/stuck/disconnect fault then restore | Purple fault state, no unbounded queue, automatic recovery | Not tested |
| LEDs | Trigger each sensor/protocol/fault event | PA0–PA3 active-low hold ~30 ms; RGB patterns match README | Not tested |
| Endurance | Run nominal traffic with both sensors for at least 30 min | No watchdog reset, queue growth, stale data or protocol loss | Not tested |

Do not perform flight acceptance until sensor orientation, scale, range offset, recovery, and 30-minute bench endurance are all documented.
