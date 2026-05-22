# Robot Hardware

## Drive System — Mecanum (4-Wheel)

| Hardware Map Name | Type | Direction |
|---|---|---|
| `frontleft` | DcMotor | REVERSE |
| `frontright` | DcMotor | FORWARD |
| `backleft` | DcMotor | REVERSE |
| `backright` | DcMotor | FORWARD |

Characterized max velocities: **54.6 in/s (forward)**, **74.5 in/s (lateral)**.

---

## Shooting System

### Flywheel (`topflywheel`, `bottomflywheel`)
- Type: DcMotorEx
- `topflywheel`: FORWARD; `bottomflywheel`: REVERSE
- Both run `RUN_USING_ENCODER` with dual PID control (`DualPidMotor.java`)
- PID: P=120.0, D=0.0, F=kf×RPM (kf ≈ 0.00380)
- RPM targets vary by shot distance:
  - 1 m: ~2375 RPM
  - 2 m: ~2725 RPM
  - Far: ~3075 RPM
- Max configurable RPM: 5800

### Turret (`turret`)
- Type: DcMotorEx
- Mode: `RUN_WITHOUT_ENCODER` (software PD loop in `Turret.java`)
- PD: kP=0.03, kD=0.0008, kF=0.043
- Travel: −160° to +140°
- Encoder ticks/rev: ~799 (145.1:1 gearbox × 110:20 stage)
- Error deadband: 0.18°

### Intake (`intake`)
- Type: DcMotorEx, Direction: REVERSE
- RPM targets depend on mode (intaking: 700–900, shooting: 600–1100)

---

## Servos

| Hardware Map Name | Function | Range | Notes |
|---|---|---|---|
| `rbstop` | Right ball stopper | 0.0 (up) → 0.3 (down) | 0.13 intermediate |
| `rhoodtilt` | Right hood angle | 0.02 (min) → 0.75 (max) | Direction: REVERSE |

Left-side equivalents (`lbstop`, `lhoodtilt`) are referenced but not actively used.

---

## Vision — Limelight 3A (`limelight`)

- Poll rate: 100 Hz
- Default pipeline: 2 (MegaTag2 AprilTag localization)
- Used for field-relative pose updates into Pedro Pathing
- Heading convention: FTC 0° = audience side; Pedro 0° = red alliance goal; offset = +90°

---

## Localization — GoBilda Pinpoint (`odo`)

- Type: GoBildaPinpointDriver
- Encoder: GoBilda 4-Bar Pod
- Forward pod Y offset: 0 mm
- Strafe pod X offset: −120 mm
- Forward encoder: FORWARD; Strafe encoder: REVERSED
- Integrated into Pedro Pathing follower

---

## LED — GoBilda Prism (`prism`)

- Type: GoBildaPrismDriver
- 10 animation layers (LAYER_0–LAYER_9), 8 artboards
- Addressable LED strip (indices 0–36 in use)

---

## Control Hubs

- All LynxModules set to bulk caching mode: **AUTO**

---

## Pedro Pathing Tuned Constants

| Parameter | Value |
|---|---|
| Robot mass | 10 kg |
| Max power | 1.0 |
| Max velocity | 0.99 |
| Max acceleration | 100 |
| Max angular velocity | 0.55 rad/s |
| Max angular acceleration | 0.5 rad/s² |
| Centripetal scaling | 0.005 |
