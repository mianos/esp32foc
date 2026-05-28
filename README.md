# ESP32 V/Hz BLDC Controller

Open-loop V/Hz BLDC controller for the MKS ESP32 FOC V1.0 board, aimed at
centrifugal pumps and other low-inertia loads where field-oriented closed-loop
control isn't worth the complexity. The drive uses FOC-style three-phase
modulation (third-harmonic-injected SVPWM, Clarke transform for stall
detection) but the velocity command is pushed straight to the field — there's
no rotor-flux estimator and no current loop. The trade-off is open-loop
pull-out at high slew rates, mitigated by a velocity ramp and a stall trip.

Single ESP32 instance, controlled over Wi-Fi via a small JSON-over-HTTP API.
Wi-Fi credentials are provisioned with ESP-Touch v2 on first boot. Firmware
updates are done over the same HTTP server with dual-OTA + rollback.

---

## Hardware

| Function       | Pin / Channel       | Notes                                                                 |
|----------------|---------------------|-----------------------------------------------------------------------|
| Phase A PWM    | GPIO 16             | → IR2104 #1 `IN`                                                      |
| Phase B PWM    | GPIO 5              | → IR2104 #2 `IN`                                                      |
| Phase C PWM    | GPIO 17             | → IR2104 #3 `IN`                                                      |
| Bridge enable  | GPIO 4              | Common SD# on all three IR2104s (active high)                         |
| Phase A sense  | GPIO 39 / ADC1_CH3  | INA240A2, 0.005 Ω shunt → 0.25 V/A, bias ~1.65 V                      |
| Phase B sense  | GPIO 36 / ADC1_CH0  | Phase C inferred as `−(Ia + Ib)`                                      |
| DC bus         | 12 V default        | Kconfig `MOTOR_VBUS_VOLTS`                                            |
| Carrier        | 20 kHz, centre-aligned | Kconfig-fixed; commutation decimated to 1 kHz in the ISR           |

Pins are configurable via `idf.py menuconfig` → *Pump Controller Configuration*.

---

### First install (wired)

The partition table includes `otadata` and two OTA app slots, so the bootloader
itself has to be flashed once over USB before OTA will work:

```sh
idf.py erase-flash
idf.py -p /dev/cu.usbserial-XXXX flash monitor
```

After the wired flash you can provision Wi-Fi from the [ESP-Touch v2 app](https://www.espressif.com/en/solutions/device-connectivity/esp-touch),
then the device joins the LAN and exposes the API.

### OTA updates after the first flash

Replace `<device>` with your unit's mDNS hostname or LAN IP (the default is
the value seeded into NVS at first boot; set via `POST /hostname`):

```sh
curl -X POST -H 'content-type: application/json' \
     -d '{"enabled":false}' http://<device>/motor
curl --data-binary @build/<project>.bin http://<device>/firmware
```

A new image lands in the inactive slot (`ota_0` ↔ `ota_1`), is verified by
`esp_ota_end` (SHA + header), then the device reboots. On the new image, if
Wi-Fi associates and the web server starts cleanly, [main.cpp](main/main.cpp)
calls `esp_ota_mark_app_valid_cancel_rollback()`. If anything panics or hangs
before that point, the bootloader (compiled with
`BOOTLOADER_APP_ROLLBACK_ENABLE`) reverts to the previous slot on next reboot.

---

## HTTP API

All endpoints accept and return JSON. The motor must be **disabled** before
`/calibrate` or `/firmware` (both return `409 Conflict` otherwise);
`Motor::disable()` is a blocking soft-stop, so when it returns the rotor is at
rest and the bridge is open.

### `GET /motor` — full status

```json
{
  "velocity_rad_s":          0,        // target (electrical rad/s)
  "current_velocity_rad_s":  0,        // post-slew commanded
  "velocity_rpm_mech":       0,        // velocity / (2π · pole_pairs) · 60
  "pole_pairs":              7,
  "voltage_v":               0,        // amplitude actually being driven
  "v_offset_v":              0.1,      // V/Hz intercept
  "v_per_rad_s":             0.001,    // V/Hz slope (back-EMF-like)
  "slew_rad_s2":             100,
  "i_mag_a":                 0.05,     // |I| from Clarke, slow EMA
  "i_bus_est_a":             0.0,
  "stall_current_a":         3,
  "enabled":                 false,
  "stalled":                 false,    // sticky; cleared on next enable
  "uptime_s":                877,
  "ia_a": 0.01, "ib_a": -0.02, "ic_a": 0.01  // raw filtered phase currents
}
```

### `POST /motor` — set any subset of fields

```json
{
  "velocity_rad_s":   500,    // electrical, clamped to MAX_VELOCITY_RAD_S
  "voltage_v":        0.15,   // v_offset_v
  "v_per_rad_s":      0.0015,
  "stall_current_a":  3.0,
  "slew_rad_s2":      150,    // 1..10000
  "pole_pairs":       7,      // RPM display only
  "enabled":          true
}
```

Tuning changes (`voltage_v`, `v_per_rad_s`, `stall_current_a`) are grouped and
persisted to NVS atomically. `slew_rad_s2`, `pole_pairs`, and pump range each
get their own NVS keys. Returns the same payload as `GET /motor`.

### `GET /calibrate` — last identification result

```json
{"valid": true, "rs_ohm": 0.62, "ls_henry": 0.00031, "v_dead": 0.18}
```

### `POST /calibrate` — measure Rs, Ls, V_dead

Refuses with `409` while the motor is enabled. Runs ~1 second of small DC
probes across the B-C winding pair (see [How identification works](#identification)).
Result is persisted to NVS and applied to the runtime alignment voltage.

### `POST /pump` — duty-style speed control

```json
{"duty": 75}     // 0–100; ≤0.5% is treated as off and disables the bridge
```

Maps duty linearly onto `[pump_min_rad_s, pump_max_rad_s]`, calls
`set_velocity` and enables the bridge. Stillerate-compatible: the optional
`"name"` field is accepted and ignored, and the response shape matches
RESTMotorController.

### `GET` / `POST /pump_range`

```json
{"min_rad_s": 50, "max_rad_s": 1000}
```

Persisted on success.

### `GET /firmware` — running image info

```json
{
  "version":   "ce343ed",
  "idf_ver":   "v6.0.1",
  "date":      "May 28 2026",
  "time":      "15:03:48",
  "partition": "ota_0"
}
```

`version` comes from `PROJECT_VER` (defaults to `git describe`).

### `POST /firmware` — push a new image

Raw `.bin` body. Streams into the inactive slot, verifies, sets it as the next
boot partition, sends `{"status":"ok","written":N,"partition":"..."}`, waits
500 ms for the response to flush, then `esp_restart()`s.

### `GET /healthz` and `/hostname`, `POST /reset`

Inherited from the [shared WebServer base](https://github.com/mianos/mianesp/tree/main/components/webserver).
`/reset` clears the stored Wi-Fi credentials and reboots into provisioning
mode; `/hostname` reads or sets the mDNS / DHCP hostname.

---

## How the drive works

This is **open-loop V/Hz**, not the usual FOC textbook block diagram. The
modulation reuses two pieces of FOC plumbing — SVPWM with third-harmonic
injection, and the Clarke transform for `|I|` — but there's no Park frame, no
`Id`/`Iq` loop, and no rotor angle estimator. The advantages are simplicity and
that current sensors only need to be good enough to detect a stall, not to
close a loop.

### V/Hz mapping

For each commanded electrical velocity ω, the drive applies a phase amplitude

```
vamp = v_offset + v_per_rad_s · |ω|
```

clamped to `MAX_VOLTAGE_AMP`. The intercept `v_offset` covers the IR2104 dead
time and winding resistance drop at standstill; the slope `v_per_rad_s` should
roughly track the motor's back-EMF constant Kₑ (V·s/rad).

### Per-enable rotor alignment

Open-loop V/Hz can't recover if the rotor isn't synced with the field at
start-up — the field zips past and the rotor stays cogged. So `enable()` runs
a 200 ms **align phase** at `ω=0`, phase=0, with a voltage calculated from the
calibration to produce ~1 A of parking current:

```
align_vamp = (V_dead + 2·Rs·1.0) / √3
```

This pulls the rotor to electrical angle 0. After 200 ms the velocity slew
takes over.

### Velocity slew

`current_velocity` ramps toward `target_velocity` at `slew_rad_s2`. A gentle
default of 100 rad/s² is set because the centrifugal load on a pump scales
with speed; ramping faster than the loaded rotor can accelerate is the
classic open-loop pull-out failure mode. The slew step is recomputed on every
tick, so a POST to `/motor` takes effect within ~1 ms.

### Stall detection

The motor task reads phase currents at the PWM-synchronized moment, runs the
Clarke α/β transform on `Ia`, `Ib`, and takes the magnitude:

```
|I| = √(Iα² + Iβ²)         (= peak phase current for balanced sinusoids)
```

That magnitude is smoothed with a slow EMA (`α = 0.005` ≈ 200 ms time
constant) so transient peaks from bias jitter don't trip. If `|I|` exceeds the
stall threshold for 100 ms while `|ω| > 200 rad/s` (so low-speed surge current
from `v_offset` doesn't false-trip), the bridge is cut, the `stalled` flag is
set, and the user has to re-enable. The arming velocity is fixed in code; the
threshold is runtime-configurable.

### Identification

`POST /calibrate` runs a two-phase routine against the B-C winding pair:

1. **Rs / V_dead** — adaptively bracket-search for a DC voltage that lands the
   current in [1.5, 2.5] A, then take five ascending probes 1.0× – 1.4× of
   that voltage and least-squares fit `V = R·I + V_dead`. The slope gives
   `R_loop`; halve it for `Rs` per phase. The intercept is the gate-driver
   dead-band loss.
2. **Ls** — drop the commutation decimation to 1 (sample at the full 20 kHz),
   apply a voltage step targeting `i_final ≈ 2 A`, capture 200 samples at
   50 µs spacing, and extract τ as the first index where
   `i ≥ (1 − 1/e) · i_final`. Then `Ls = R_loop · τ / 2`.

The result is informational for now — only `V_dead` and `Rs` are wired into
runtime control (sizing the alignment voltage). `Ls` is logged so it can be
used later for a current loop.

---

## Architecture: how it stays real-time

The motor commutation has to run at a rock-steady rate regardless of Wi-Fi,
HTTP requests, NVS writes, or OTA flash writes. Three things make that work:
**core isolation**, **IRAM + cache-safe ISR**, and **lock-free task→ISR
publish**.

### Core isolation

```
  Core 0 (PRO):                Core 1 (APP):
    LWIP / TCP / HTTP server     Motor commutation task (1 kHz)
    Wi-Fi MAC                    MCPWM carrier-peak ISR  (20 kHz, decimated)
    NVS / flash operations       INA240 ADC reads
    OTA stream
```

The MCPWM ISR is allocated on core 1 by pinning the `mcpwm_init` task that
calls `mcpwm_timer_register_event_callbacks` ([motor.cpp:272-278](main/motor.cpp#L272-L278)).
The motor task is pinned to the same core. Wi-Fi traffic and HTTP request
processing on core 0 cannot delay or drop carrier interrupts.

### IRAM + cache-safe

Flash cache disable windows are the enemy of real-time on ESP32 — every NVS
commit, every OTA write, every SPI flash erase parks the cache for a few
hundred microseconds. Code in flash can't execute during those windows.

The carrier-peak ISR (`Motor::on_pwm_peak`) is `IRAM_ATTR` so it stays
resident, and the one MCPWM driver function it calls
(`mcpwm_comparator_set_compare_value`) is pulled into IRAM via
`CONFIG_MCPWM_CTRL_FUNC_IN_IRAM`. The ISR itself is registered as
cache-safe via `CONFIG_MCPWM_ISR_CACHE_SAFE`, which keeps it unmasked
during cache-off windows. Together they let the field keep rotating during
NVS writes, OTA streams, and any other flash activity ([sdkconfig.defaults](sdkconfig.defaults)).

The motor *task* is flash-resident and will stall during a cache-off window —
that's fine, because the ISR keeps integrating the angle at the last
published rate.

### Lock-free task → ISR handoff

The carrier ISR runs every 50 µs and the commutation tick fires every 1 ms,
so heavy float math has to happen *outside* the ISR. The motor task computes:

- `phase_inc` — the integer DDS increment per tick (uint32 spans a full
  electrical revolution, so a 120° offset is exactly `2³²/3`)
- `amp_q` — Q15 modulation amplitude clamped to the third-harmonic
  headroom (~0.55)

and publishes them via `std::atomic<uint32_t>` / `std::atomic<int32_t>`
([motor.cpp:155-170](main/motor.cpp#L155-L170)). The ISR reads them with
relaxed ordering and integrates:

```cpp
isr_phase += phase_inc;
write_duties_ticks(
    phase_duty_ticks(isr_phase,                    amp_q),
    phase_duty_ticks(isr_phase + PHASE_OFFSET_120, amp_q),
    phase_duty_ticks(isr_phase + PHASE_OFFSET_240, amp_q));
```

`phase_duty_ticks` evaluates `sin(θ) + (1/6)·sin(3θ)` in Q15 with a
256-entry LUT, scales by `amp_q`, and converts to PWM ticks ([motor.cpp:135-147](main/motor.cpp#L135-L147)).
All integer — float in an ESP32 ISR is undefined behaviour because the FPU
isn't saved across interrupts.

Aligned 32-bit loads and stores are atomic on Xtensa LX6, so plain
`std::atomic<T>` for T ∈ {`uint32_t`, `int32_t`, `float`, `bool`} compiles to
a single `l32i.n` / `s32i.n` — no locking, no memory barriers beyond the
acquire/release on `phase_epoch_`.

### Enable-edge synchronization

The task signals the enable edge to the ISR via a monotonic `phase_epoch_`
counter (acquire/release ordered). The ISR notices the bump, zeroes
`isr_phase_`, and the field parks at electrical angle 0 during the align
phase — no race between "enable command" and "first compare write."

### Carrier decimation

The MCPWM timer runs at 20 kHz centre-aligned. Commutation only needs to
update at ~1 kHz, so the ISR keeps an integer divider (`commutation_decimation_`,
default 20) and skips most calls. The identification routine drops it to 1
temporarily for full-rate sampling during the `Ls` step.

### Other tweaks worth knowing about

- **Wi-Fi power save off**: `esp_wifi_set_ps(WIFI_PS_NONE)` on connect, in
  [WifiConnection.cpp](main/WifiConnection.cpp). DTIM cycles couple as an
  audible 1 Hz tick into the motor; ~80 mA extra is irrelevant against a
  2 A motor.
- **Bias re-zero on every enable**: INA240+shunt drifts with temperature.
  `CurrentSense::calibrate_bias` runs with the gate off at every `enable()`
  and every `identify()` so `|I|` is meaningful when the trip decision is
  made.
- **Soft-stop on disable**: `Motor::disable()` ramps velocity to 0 before
  cutting the gate, bounded at 5 s. Abrupt tristating with a spinning rotor
  freewheels the back-EMF through body diodes and emits a switching
  transient that couples noise into the UART.

---

## NVS keys (survive OTA & re-flash)

The partition table keeps `nvs` at its built-in offset (`0x9000`, size
`0x6000`) regardless of whether the layout is single-app or dual-OTA, so the
following settings survive any firmware change short of `idf.py erase-flash`:

| Key group        | Set by                          |
|------------------|---------------------------------|
| `host_name`      | provisioning or `/hostname`     |
| Wi-Fi creds      | ESP-Touch v2 provisioning       |
| `rs`, `ls`, `vdead`, `cal_valid` | `POST /calibrate`         |
| `v_off`, `v_per`, `i_st`         | `POST /motor` (tuning)    |
| `slew`           | `POST /motor {"slew_rad_s2":N}` |
| `pump_min`, `pump_max` | `POST /pump_range`        |
| `pole_pairs`     | `POST /motor {"pole_pairs":N}`  |

---

Wi-Fi, NVS, HTTP server, and JSON parsing come from
[mianos/mianesp](https://github.com/mianos/mianesp) (declared in
[main/idf_component.yml](main/idf_component.yml)).
