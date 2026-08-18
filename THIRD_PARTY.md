# Third-party software

FlowCAN source is MIT-licensed. The following vendored components keep their upstream terms.

| Component | Pinned version / revision | Source | License |
|---|---|---|---|
| FreeRTOS-Kernel | V11.3.0, commit `9b777ae5c5b8e9e456065a00294d1e5f5f9facf5` | [FreeRTOS/FreeRTOS-Kernel](https://github.com/FreeRTOS/FreeRTOS-Kernel/tree/V11.3.0) | MIT, see `third_party/freertos/LICENSE.md` |
| Artery AT32F415 BSP | v2.0.7 (2022-08-16 headers) | Artery AT32F415 firmware library | Artery BSP notice in every source file; use is authorized with Artery MCUs |
| libcanard | v0.2 local AM32 vendor snapshot; `canard.c` SHA-256 `5fc1e768102dd53ffbb64741e7ff3888a5ea8a12736f7d302404cd7ca9ec020f`, `canard.h` SHA-256 `dacac3f613cbfc0d6adda6bf31b928ef6d80423ed1ce7361c94e2434ddbb391e` | [DroneCAN/libcanard](https://github.com/dronecan/libcanard) | MIT, see `third_party/libcanard/LICENSE` |
| VL53L1X ULD core | API 3.5.0, repository commit `1c474582e72733bd7f4df83a266bad7d3b83bbf5` | [rneurink/VL53L1X_ULD](https://github.com/rneurink/VL53L1X_ULD), derived from ST STSW-IMG009 | BSD-3-Clause, see `third_party/st-vl53l1x/LICENSE` and source headers |
| PMW3901 register sequence | commit `d322d98d6f61757352d11d922cd194539e165231` | [Bitcraze/Bitcraze_PMW3901](https://github.com/bitcraze/Bitcraze_PMW3901) | MIT, see `third_party/PMW3901-LICENSE` |
| DroneCAN DSDL | commit `b4653c7abc3c47cb31b16efa24ea755232774756` | [DroneCAN/DSDL](https://github.com/dronecan/DSDL) | MIT, see `third_party/dronecan/LICENSE` |
| dronecan_dsdlc | generator commit `431170fa4bfe2212b516b8f33bdc796267907f1c` | [DroneCAN/dronecan_dsdlc](https://github.com/DroneCAN/dronecan_dsdlc) | Used to generate checked-in C bindings |

Only `tasks.c`, `queue.c`, `list.c`, `event_groups.c`, public headers, and the GCC ARM_CM3 port are retained from FreeRTOS. No `heap_*.c`, software timer, stream-buffer, or co-routine implementation is compiled. The Arduino-specific VL53L1X platform files are not used. The ULD core was renamed from C++ to C, one C++ bit-pattern cast was converted to C11 `memcpy`, and the legacy unbounded data-ready loop has a finite error-aware limit; these adaptations are documented in source. FlowCAN's runtime initialization writes the ULD configuration incrementally instead of calling the blocking helper. The AT32 CAN reset routine has an explicit unused-parameter cast so the pinned BSP builds under `-Werror`. Generated DroneCAN bindings are checked in so firmware builds do not download code.
