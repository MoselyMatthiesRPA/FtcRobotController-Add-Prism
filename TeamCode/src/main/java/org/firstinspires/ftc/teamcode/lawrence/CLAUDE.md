# Lawrence TeamCode — FTC Decode Season

## What This Robot Does
Ball shooter. The turret rotates to aim at the goal, flywheel launches balls, hood adjusts launch angle, intake feeds balls into the flywheel. Limelight 3A provides field-relative pose correction via MegaTag2 AprilTags. Pedro Pathing + GoBilda Pinpoint odometry handles autonomous movement.

## Package Layout
```
constants/   — HardwareNames, RobotConstants (@Configurable), FieldPositions
subsystems/  — Flywheel, Turret  (both need update() called every loop)
vision/      — LimelightLocalizer, HeadingConverter
teleop/      — CompetitionDrive
auto/        — {Alliance}{Zone}{BallCount}.java  (e.g. RedFar3, BlueGoal12)
```

## Coordinate Systems (Critical)
Two frames in use — mixing them is the #1 source of bugs.

| | Origin | 0° heading | +X | +Y |
|---|---|---|---|---|
| **FTC / Limelight** | Field center | Audience wall | Toward audience | Red→Blue |
| **Pedro Pathing** | Blue corner | Red alliance goal | FTC +Y direction | FTC −X direction |

Conversion (in `HeadingConverter.java`):
- FTC heading = Pedro heading + 90°
- `pedroX = ftcY_in + 72`, `pedroY = -ftcX_in + 72`

Always pass `HeadingConverter.pedroToLimelightDeg()` into `limelight.updateRobotOrientation()` — never raw Pedro heading.

## Physical Constraints That Explain Code Choices

**Stopper servo**: `STOPPER_DOWN = 0.13`, not 0.3. The hardware range goes to 0.3 but that position jams rings. 0.13 is the tested safe "feed" position.

**Hood tilt**: `TILT_AT_FAR = 0.70` intentionally exceeds `MAX_TILT = 0.65`. The value is clamped in `CompetitionDrive.loop()`. Was retained from physical calibration where unclamped slightly outperformed.

**Left-side servos** (`lbstop`, `lhoodtilt`): Physically wired but not used. Only the right side is active.

**Turret reset**: `new Turret(hardwareMap, true)` resets the encoder. Do this in every OpMode init — the turret has no absolute encoder and must start from 0.

**Turret fallback angle in auto**: `turretFallBackAngle = 65.5°` (Blue/positive) or `-65.5°` (Red/negative) is a pre-aimed angle calibrated for the far shoot position. It is not derived from field geometry — it was tuned by watching the physical turret aim at the goal from that pose.

## Shooting Distance Model
Two-point linear interpolation between 1 m and 2 m calibration points, then a flat FAR constant beyond `FAR_THRESHOLD_IN = 105"`. The raw Pedro-reported distance to the goal was measured to disagree with tape-measured distance, so `RobotConstants` contains a linear correction (`DIST_A`, `DIST_B`) fit to those two physical measurements. If shots are off at a new venue, re-measure at 1 m and 2 m and update `RAW_AT_1M_IN` / `RAW_AT_2M_IN`.

## Limelight / MT2 Pose Fusion
- **Must seed yaw before the first MT2 solve** (done in `init()` and `start()`). Without a correct heading seed, MT2 can return a mirrored position on the opposite side of the field — this happened in physical testing.
- Pipeline 2 = MegaTag2. Pipeline 0/1 used in some autos (verify before match).
- `MIN_POSE_JUMP_IN = 0.5"` — filters jitter when stationary (localizer was nudging the pose every frame).
- `MAX_POSE_JUMP_IN = 24"` — rejects wild outliers from bad tag solves.
- Pose is injected into Pedro at Pedro heading (not FTC heading) — `follower.setPose()` takes Pedro-frame radians.

## Live Tuning
`@Configurable` (bylazar Configurables) exposes static fields to the dashboard at runtime. `RobotConstants`, `Turret`, `Flywheel`, `CompetitionDrive`, and each Auto OpMode all use it. Change values on the dashboard; they persist only for the session — update the source to make them permanent.

## Auto Naming Convention
`{Alliance}{Zone}{BallCount}` — e.g. `RedFar3` = Red alliance, far zone (far from goal), shoot 3 rings. `BlueGoal12` = Blue alliance, goal zone (near goal), 12 rings. `RevampedAuto` is a standalone test/dev auto not tied to a specific match config.

## Docs Reference
Detailed tuning and SDK notes in `/docs/`:
- `pedro-pathing-constants.md` — tuned follower/localizer values
- `limelight-megatag2.md` — MT2 filtering and heading conventions
- `limelight-ftc-programming.md` — FTC SDK Limelight API
- `gobilda-pinpoint-user-guide.pdf` — odometry pod wiring/offsets
- `ftc-pidf-coefficients.md` — flywheel PIDF tuning reference

Hardware map names and servo/motor directions: see `hardware.md` at the repo root.
