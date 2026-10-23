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
    ├── hexmeow_canfd.{h,c}         # shared transport: SocketCAN-FD
    ├── hexmeow_sdo.{h,c}           # shared transport: expedited SDO master
    │
    ├── hexmeow_common.h            # hexmeow (CiA402) family
    ├── hexmeow_mit_target.{h,c}
    ├── hexmeow_motor_init.{h,c}
    ├── hexmeow_motor_ctrl.{h,c}
    │
    ├── meow_common.h                 # Meow Motor (0x41xx/0x45xx) family
    ├── meow_motor_init.{h,c}
    ├── meow_motor_ctrl.{h,c}
    │
    └── apps/
        ├── demo_mit.c
        ├── demo_velocity.c
        ├── demo_regen.c
        └── demo_meow_mit.c
```

There is **no external CANopen stack** — see the README for why. The
SDO master is a tiny custom implementation in `hexmeow_sdo.{h,c}`,
and the high-rate control path goes straight to a SocketCAN-FD
socket.

The two motor families share only those two transport modules. The
`hexmeow_*` control modules assume the CiA402 profile and the
`meow_*` ones assume the Meow Motor object dictionary; neither includes
the other, and each demo links only the pair it needs.

## src/ — module by module

### `hexmeow_common.h`

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

### `hexmeow_canfd.{h,c}`

Thin SocketCAN-FD wrappers. One socket is opened per program with
`CAN_RAW_FD_FRAMES` enabled, allowing it to send/receive both classic
(`struct can_frame`, 16 B) and FD (`struct canfd_frame`, 72 B) frames.

Key exports:

- `hexmeow_canfd_open(ifname)` — bind to a CAN-FD interface (aborts on
  failure).
- `hexmeow_canfd_send_classic(fd, cob, data, len)` — used for NMT
  (`0x000`) and master heartbeat (`0x710`).
- `hexmeow_canfd_send_fd(fd, cob, data, len)` — used for the shared
  RPDO (`0x190`). Pads to the nearest legal FD DLC (`0..8, 12, 16, 20,
  24, 32, 48, 64`) and always sets `CANFD_BRS` (some FD controllers
  will not transmit an FD frame whose data phase is at the
  arbitration bitrate, which would silently wedge the kernel TX
  queue).
- `hexmeow_canfd_recv(fd, &frame, &is_fd)` — non-blocking
  (`MSG_DONTWAIT`) read; returns `-2` if the kernel queue is empty.
  Required because TPDO1 ticks at 1 ms and a blocking read with
  `SO_RCVTIMEO=1ms` is ambushed by the next arriving frame and never
  returns.

### `hexmeow_sdo.{h,c}`

Minimal expedited-only SDO client used during initialization. It uses a
*separate* classic-CAN socket with a kernel-side filter limited to
`0x580..0x5FF` (SDO server responses) so TPDO/heartbeat traffic from the
control loop cannot poison the SDO reply queue.

Supports only expedited transfers (≤ 4 byte payload), which is all the
hexmeow init touches:

- Byte: `hexmeow_sdo_dl_u8 / _ul_u8`
- Word: `_dl_u16 / _ul_u16`
- DWord/float: `_dl_u32 / _dl_f32 / _ul_u32 / _ul_f32`
- Signed byte: `_dl_i8`

Each call sends the SDO request, waits for a matching server response
with `SO_RCVTIMEO = HEXMEOW_SDO_TIMEOUT_MS` (500 ms), and returns one
of the `HEXMEOW_SDO_ERR_*` codes on failure (timeout, server abort,
protocol violation). Callers in `hexmeow_motor_init.c` use the
`SDO_DL` / `SDO_UL` macros which abort the whole program on any error
— this is a demo with no recovery path.

### `hexmeow_mit_target.{h,c}`

MIT-mode 8-byte target packing, matching the layout the firmware
expects in object dictionary `2004h`:

- 16-bit unsigned position over `[position_min, position_max]` (Rev)
- 12-bit unsigned velocity, kp, kd, torque over their respective ranges

The packed layout is:

```
[ torque(12) | kd(12) | kp(12) | velocity(12) | position(16) ]
  → 8 little-endian bytes
```

The `hexmeow_mit_mapping_t` parameter is shared per motor-set: the
demo sets it once at startup and the same ranges are written into the
motor's `2004h-04..0D` subindices during init, so the encoder side and
the decoder side agree.

### `hexmeow_motor_init.{h,c}`

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

### `hexmeow_motor_ctrl.{h,c}`

The runtime: **one pthread per process**, living for the lifetime of the
process. There is no stop / join handle by design — see the shutdown
contract in the README.

**Control thread (`thread_main`).** One tight loop running at
`HEXMEOW_RPDO_PERIOD_MS` (= 2 ms) cadence. Each iteration, in order:

1. **HB-if-due.** If at least `HEXMEOW_MASTER_HB_PERIOD_MS` (50 ms)
   has elapsed since the last heartbeat, send one master heartbeat on
   cob `0x710` (payload `[0x05]`) via the classic-CAN write path. The
   first iteration's `last_hb_ms = 0` makes this fire immediately, so
   the very first frame the thread puts on the bus is a heartbeat.
2. **RPDO.** Build and send one shared RPDO on cob `0x190`:
   - MIT mode: `8 × count` bytes, each motor's slot produced by
     `hexmeow_mit_target_pack`.
   - Velocity mode: `6 × count` bytes,
     `[u16 6072h][f32 60FFh]` per motor.
3. **Drain TPDOs.** `drain_rx` calls `hexmeow_canfd_recv` in a tight
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

`hexmeow_ctrl_start` simply spawns this thread; there is no longer a
synchronous initial heartbeat in the calling thread (the loop itself
sends one immediately).

Targets are written via `hexmeow_ctrl_set_mit_target` /
`hexmeow_ctrl_set_velocity_target`; the latest written value is
picked up by the next RPDO tick (within ≤ 2 ms).

After `hexmeow_ctrl_init`, every target is zeroed — meaning the RPDO
is benign and the motor will not spin. The application is expected to
call `hexmeow_ctrl_start`, wait for the heartbeat warm-up
(`HB_WARMUP_MS = 100 ms`), and only *then* write actual targets.

**Verbose logging.** The global `int hexmeow_verbose` (declared in
`hexmeow_common.h`, defined in `hexmeow_motor_ctrl.c`) is flipped to
`1` by the demos when `--verbose`/`-v` is passed. When set, the control
thread emits short `[v] <t_ms> <event>` lines on stderr: per-heartbeat,
per-drain (with `T1=/T2=/other=` counts), every ~50th RPDO (to avoid
flooding 500 lines/sec), and a "fell behind by N ms" warning when the
loop slips badly.

### `apps/demo_mit.c`

Top-level program for MIT mode (mode 5). Sequence:

1. Strip `--verbose`/`-v` from `argv` (sets the global
   `hexmeow_verbose`), then parse the remaining positional args
   `iface motor_count [target_rev_per_s] [kd] [torque_permille]`.
2. Install `SIGINT` / `SIGTERM` handler that calls `_exit(0)` directly.
3. Open the CAN-FD and SDO sockets.
4. Run `hexmeow_motor_set_init` for every motor.
5. `hexmeow_ctrl_init` (targets zero).
6. `hexmeow_ctrl_start` — the single control thread begins emitting
   heartbeats (50 ms) and zero-target RPDOs (2 ms).
7. `usleep(100 ms)` for heartbeat warm-up.
8. `hexmeow_ctrl_set_mit_target` per motor with the requested
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

### `meow_common.h`

Constants for the Meow Motor family: the `0x1018` identity pair, the
CiA301 COB-ID bases, every object-dictionary index the demo touches
(`0x3000` fillers, `0x4102` MIT record, `0x4104` temperatures, `0x4401`
mode, `0x4402` mode display, `0x453F` error, `0x4564` position,
`0x4572`/`0x4576`/`0x4577` torque, `0x4713` timestamp), the `0x4401`
mode codes, the TPDO/RPDO shapes and the loop cadences.

The header carries the reasoning for the two RPDO payload lengths: both
20 (full) and 12 (slim) are exact CAN-FD DLCs, so the frame length
matches the mapped length precisely rather than relying on the receiver
tolerating trailing bytes.

### `meow_motor_init.{h,c}`

Single-motor SDO bring-up. Unlike `hexmeow_motor_init.c` there is no
one-to-many slicing: the motor keeps its own default RPDO1 COB-ID
(`0x200 + node_id`) and the whole payload belongs to it.

Order: NMT pre-operational → `0x1018` identity (hard-fail on the wrong
vendor, warn on an unknown product code) → clear a latched fault via
`0x4401 = 0xFF` if `0x4402` reports one → disable (`0x4401 = 0`) → read
peak torque (`0x4576`) and the MIT Kp/Kd scale (`0x4102:07`) → TPDO1,
TPDO2 and RPDO1 mapping → torque ceiling (`0x4572`) → consumer
heartbeat (`0x1016:01`) → zero all six MIT fields → MIT mode
(`0x4401 = 4`) → NMT operational.

`map_entry()` writes the `0x1600` mapping entries through the raw SDO
call rather than the `dl_u32` helper so it can report the abort code —
this is the one step where firmware builds plausibly differ, and the
error message points at `--slim` when `0x4102:01` or `:04` are refused.

### `meow_motor_ctrl.{h,c}`

The runtime loop, structurally identical to `hexmeow_motor_ctrl.c`:
one thread, `HB-if-due → RPDO → drain → sleep` on a 2 ms tick, with the
heartbeat and the RPDO deliberately sharing a thread so a wedged TX
path trips the motor's watchdog instead of leaving it on a stale
target.

Differences from the hexmeow version:

- Position (`0x4564`) is decoded as signed **Q8.24** and is already
  multi-turn over ±128 Rev, so there is no single-turn wrap accumulator.
- The timestamp (`0x4713`) is 16 bits, not 32.
- `drain_rx()` counts received PDOs by frame kind (`rx_fd` /
  `rx_classic`). That counter is the demo's evidence that the CAN-FD
  receive path works, and is what the per-second telemetry reports.
- `meow_mit_target_pack()` emits either the 20-byte or the 12-byte
  layout, zeroing the filler bytes.
- `meow_gain_to_raw()` converts a physical gain in Nm per (Rev or
  Rev/s) into the dimensionless raw value `0x4102:04`/`:05` expect,
  using the factory scale factor and saturating at 65535.

### `apps/demo_meow_mit.c`

Drives one Meow Motor as a velocity source with a torque ceiling: `Kp`
and `Pdes` stay at zero, `Vdes` and `Kd` do the work, and the CLI
`torque_permille` is written to both the Kp/Kd clamp (`0x4102:06`) and
the motor-side ceiling (`0x4572`) so nothing it commands can exceed
that number.

Same lifecycle as `demo_mit.c` — init with no heartbeats, start the
control thread, 100 ms warm-up, then apply the target — with telemetry
extended to report the FD/classic split of received PDOs. `--verbose`
additionally dumps the exact RPDO payload bytes once, for comparison
against a `candump` trace.

## Outside src/

### `Makefile`

GCC-based; no autotools, no CMake. Builds `build/demo_mit`,
`build/demo_velocity`, `build/demo_regen` and `build/demo_meow_mit`.
Objects are grouped into `COMMON_OBJS` (the two transport modules),
`HEXMEOW_OBJS` and `MEOW_OBJS`, and each binary links only the groups
it needs — so the CiA402 code and the Meow Motor code never end up in
the same executable. Honors environment variables `CC`, `CFLAGS_EXTRA`,
`LDFLAGS_EXTRA`, and `BUILD_DIR` for cross compilation (see the README
for examples).
