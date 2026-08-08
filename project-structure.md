# Project structure

A per-file tour of the demo. Start with [`README.md`](README.md) for the
overall purpose, hard-coded presets, and run instructions; this document
exists to make it easy to navigate the source.

## Top-level layout

```
.
├── Makefile                         # native + cross build
├── README.md                        # overall guide (presets, run, lifecycle)
├── project-structure.md             # ← this file
└── src/
    ├── hexfellow_common.h
    ├── hexfellow_canfd.{h,c}
    ├── hexfellow_sdo.{h,c}
    ├── hexfellow_mit_target.{h,c}
    ├── hexfellow_motor_init.{h,c}
    ├── hexfellow_motor_ctrl.{h,c}
    └── apps/
        ├── demo_mit.c
        ├── demo_velocity.c
        └── demo_regen.c
```

There is **no external CANopen stack** — see the README for why. The
SDO master is a tiny custom implementation in `hexfellow_sdo.{h,c}`,
and the high-rate control path goes straight to a SocketCAN-FD
socket.

## src/ — module by module

### `hexfellow_common.h`

Hard-coded protocol constants shared by every other module:

- Master node ID (`0x10`) and matching heartbeat COB-ID (`0x710`).
- Maximum motor count (`8`), required firmware version (`8`), expected
  factory UID (`0x4859444C`).
- Heartbeat period (50 ms) and motor consumer-heartbeat timeout (250 ms).
- TPDO event-timer / inhibit values, RPDO COB-ID (`0x190`).
- CiA301 COB-ID bases and NMT command bytes.
- CiA402 modes-of-operation values used by the demos (`3` = profile
  velocity, `5` = MIT).

If your setup deviates from any of these, edit this header.

### `hexfellow_canfd.{h,c}`

Thin SocketCAN-FD wrappers. One socket is opened per program with
`CAN_RAW_FD_FRAMES` enabled, allowing it to send/receive both classic
(`struct can_frame`, 16 B) and FD (`struct canfd_frame`, 72 B) frames.

Key exports:

- `hexfellow_canfd_open(ifname)` — bind to a CAN-FD interface (aborts on
  failure).
- `hexfellow_canfd_send_classic(fd, cob, data, len)` — used for NMT
  (`0x000`) and master heartbeat (`0x710`).
- `hexfellow_canfd_send_fd(fd, cob, data, len)` — used for the shared
  RPDO (`0x190`). Pads to the nearest legal FD DLC (`0..8, 12, 16, 20,
  24, 32, 48, 64`) and always sets `CANFD_BRS` (some FD controllers
  will not transmit an FD frame whose data phase is at the
  arbitration bitrate, which would silently wedge the kernel TX
  queue).
- `hexfellow_canfd_recv(fd, &frame, &is_fd)` — non-blocking
  (`MSG_DONTWAIT`) read; returns `-2` if the kernel queue is empty.
  Required because TPDO1 ticks at 1 ms and a blocking read with
  `SO_RCVTIMEO=1ms` is ambushed by the next arriving frame and never
  returns.

### `hexfellow_sdo.{h,c}`

Minimal expedited-only SDO client used during initialization. It uses a
*separate* classic-CAN socket with a kernel-side filter limited to
`0x580..0x5FF` (SDO server responses) so TPDO/heartbeat traffic from the
control loop cannot poison the SDO reply queue.

Supports only expedited transfers (≤ 4 byte payload), which is all the
hexfellow init touches:

- Byte: `hexfellow_sdo_dl_u8 / _ul_u8`
- Word: `_dl_u16 / _ul_u16`
- DWord/float: `_dl_u32 / _dl_f32 / _ul_u32 / _ul_f32`
- Signed byte: `_dl_i8`

Each call sends the SDO request, waits for a matching server response
with `SO_RCVTIMEO = HEXFELLOW_SDO_TIMEOUT_MS` (500 ms), and returns one
of the `HEXFELLOW_SDO_ERR_*` codes on failure (timeout, server abort,
protocol violation). Callers in `hexfellow_motor_init.c` use the
`SDO_DL` / `SDO_UL` macros which abort the whole program on any error
— this is a demo with no recovery path.

### `hexfellow_mit_target.{h,c}`

MIT-mode 8-byte target packing, matching the layout the firmware
expects in object dictionary `2004h`:

- 16-bit unsigned position over `[position_min, position_max]` (Rev)
- 12-bit unsigned velocity, kp, kd, torque over their respective ranges

The packed layout is:

```
[ torque(12) | kd(12) | kp(12) | velocity(12) | position(16) ]
  → 8 little-endian bytes
```

The `hexfellow_mit_mapping_t` parameter is shared per motor-set: the
demo sets it once at startup and the same ranges are written into the
motor's `2004h-04..0D` subindices during init, so the encoder side and
the decoder side agree.

### `hexfellow_motor_init.{h,c}`

The "long" SDO bring-up script. Per motor:

1. NMT pre-operational (so the motor accepts re-mapping).
2. Validate `1018h-01` (factory UID) and `1018h-03` (firmware == 8).
3. Read `1018h-04` (serial number) and `6076h` (peak torque, REAL32).
4. Reset CiA402 state machine (`6040h = 0`, then two `0x80` fault
   resets).
5. Disable consumer-heartbeat monitoring (`1016h-01 = 0`) while we
   reconfigure.
6. Enable short-circuit braking on disable (`2040h = 1`).
7. Per-mode max-torque (`6072h`).
8. (MIT only) `2004h-0E` KP/KD output limit and `2004h-04..0D`
   target ranges; pre-load `2004h-02/03` with a zero MIT frame; enable
   the compressed MIT control parameter (`2004h-01 = 1`).
9. Configure TPDO1: disable → clear mapping → write four entries
   (position f32, timestamp u32, torque i16, error u16) → set type 255,
   inhibit 0.5 ms, event 1 ms → enable.
10. Configure TPDO2: status word, drv temp, mtr temp, control word,
    error code; type 255, inhibit 19 ms, event 20 ms.
11. RPDO1 one-to-many mapping. Every motor's RPDO COB-ID is rewritten
    to the shared `0x190`; the mapping table marks this motor's bytes
    as its real targets (`2004h-02 / -03` for MIT, `6072h / 60FFh` for
    velocity) and the other motors' slots as padding objects
    (`3000h-02 / -03`).
12. Re-enable consumer-heartbeat monitoring of master `0x10` with the
    250 ms timeout.
13. Mode of operation (`6060h = 5` for MIT, `3` for profile velocity).
14. CiA402 ramp `6 → 7 → 0x0F`.
15. NMT operational.

Errors at any step abort the program (UID mismatch, version != 8, SDO
timeout, ...).

Note: **no master heartbeat is sent during this phase.** The motor's
consumer-heartbeat watchdog (1016h) is only *armed* once it sees the
first heartbeat — so deferring all heartbeat traffic until after every
motor is fully configured is what keeps a slow init from tripping
`0x8130` "Heartbeat error" on the motors that finished early.

### `hexfellow_motor_ctrl.{h,c}`

The runtime: **one pthread per process**, living for the lifetime of the
process. There is no stop / join handle by design — see the shutdown
contract in the README.

**Control thread (`thread_main`).** One tight loop running at
`HEXFELLOW_RPDO_PERIOD_MS` (= 2 ms) cadence. Each iteration, in order:

1. **HB-if-due.** If at least `HEXFELLOW_MASTER_HB_PERIOD_MS` (50 ms)
   has elapsed since the last heartbeat, send one master heartbeat on
   cob `0x710` (payload `[0x05]`) via the classic-CAN write path. The
   first iteration's `last_hb_ms = 0` makes this fire immediately, so
   the very first frame the thread puts on the bus is a heartbeat.
2. **RPDO.** Build and send one shared RPDO on cob `0x190`:
   - MIT mode: `8 × count` bytes, each motor's slot produced by
     `hexfellow_mit_target_pack`.
   - Velocity mode: `6 × count` bytes,
     `[u16 6072h][f32 60FFh]` per motor.
3. **Drain TPDOs.** `drain_rx` calls `hexfellow_canfd_recv` in a tight
   loop until the kernel queue is empty (non-blocking). Decoded
   TPDO1 / TPDO2 frames update per-motor state under a mutex:
   - TPDO1 → position (with software multi-turn extension), timestamp,
     torque (permille), error code.
   - TPDO2 → status word, driver temp, motor temp, control word.
4. **Sleep to next tick.** `nanosleep` until `now_ms() ≥
   next_tick_ms + 2 ms`. If the loop has fallen behind, the tick is
   resynced to the current time (and a `[v]` line is printed if
   verbose is on).

Putting the heartbeat and the RPDO in the same thread is **intentional
coupling**: if anything in this loop wedges (a blocked `write()` on the
FD socket, for instance), the heartbeat stops too, the motor's 250 ms
consumer-heartbeat watchdog trips, and the failure is visible — the
motor stops cleanly rather than spinning on a stale target.

`hexfellow_ctrl_start` simply spawns this thread; there is no longer a
synchronous initial heartbeat in the calling thread (the loop itself
sends one immediately).

Targets are written via `hexfellow_ctrl_set_mit_target` /
`hexfellow_ctrl_set_velocity_target`; the latest written value is
picked up by the next RPDO tick (within ≤ 2 ms).

After `hexfellow_ctrl_init`, every target is zeroed — meaning the RPDO
is benign and the motor will not spin. The application is expected to
call `hexfellow_ctrl_start`, wait for the heartbeat warm-up
(`HB_WARMUP_MS = 100 ms`), and only *then* write actual targets.

**Verbose logging.** The global `int hexfellow_verbose` (declared in
`hexfellow_common.h`, defined in `hexfellow_motor_ctrl.c`) is flipped to
`1` by the demos when `--verbose`/`-v` is passed. When set, the control
thread emits short `[v] <t_ms> <event>` lines on stderr: per-heartbeat,
per-drain (with `T1=/T2=/other=` counts), every ~50th RPDO (to avoid
flooding 500 lines/sec), and a "fell behind by N ms" warning when the
loop slips badly.

### `apps/demo_mit.c`

Top-level program for MIT mode (mode 5). Sequence:

1. Strip `--verbose`/`-v` from `argv` (sets the global
   `hexfellow_verbose`), then parse the remaining positional args
   `iface motor_count [target_rev_per_s] [kd] [torque_permille]`.
2. Install `SIGINT` / `SIGTERM` handler that calls `_exit(0)` directly.
3. Open the CAN-FD and SDO sockets.
4. Run `hexfellow_motor_set_init` for every motor.
5. `hexfellow_ctrl_init` (targets zero).
6. `hexfellow_ctrl_start` — the single control thread begins emitting
   heartbeats (50 ms) and zero-target RPDOs (2 ms).
7. `usleep(100 ms)` for heartbeat warm-up.
8. `hexfellow_ctrl_set_mit_target` per motor with the requested
   velocity (kp = 0, kd from CLI, torque feed-forward = 0).
9. Loop forever printing per-second telemetry; exit only on signal.

### `apps/demo_velocity.c`

Same skeleton as `demo_mit.c`, but configures every motor for profile
velocity (`6060h = 3`) and sends `[u16 6072h][f32 60FFh]` per motor in
each RPDO instead of the MIT 8-byte packed frame.

### `apps/demo_regen.c`

Same init skeleton as `demo_mit.c` (MIT mode), but instead of a
constant velocity the main thread runs a 2 ms waveform generator that
slews the shared velocity command through a trapezoidal
`+peak → cruise → -peak → cruise → ...` cycle. Motoring ramps use
`accel_rev_s2`, braking ramps use `decel_rev_s2`; every motor gets the
identical command so all regen currents hit the DC bus at once — the
worst case for back-EMF absorption hardware, which is what this demo
exists to stress. Unlike `demo_mit`, the MIT KP/KD output limit
(`2004h-0E`) is set to the CLI `torque_permille` instead of the
conservative hard-coded 770. Telemetry adds a measured velocity
(position delta per telemetry interval) next to the commanded one, and
phase transitions are logged with `>>>` markers for scope correlation.

## Outside src/

### `Makefile`

GCC-based; no autotools, no CMake. Builds `build/demo_mit`,
`build/demo_velocity` and `build/demo_regen`. Honors environment
variables `CC`, `CFLAGS_EXTRA`, `LDFLAGS_EXTRA`, and `BUILD_DIR` for
cross compilation (see the README for examples).
