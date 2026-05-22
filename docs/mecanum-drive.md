# Mecanum Drive — FTC

Source: https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html

## Overview

Mecanum drivetrains enable holonomic movement — robots can travel in any direction while simultaneously rotating. Mecanum wheels feature rollers oriented at 45-degree angles. By combining forces from four strategically oriented wheels, robots achieve omnidirectional movement through vector addition and cancellation.

The standard configuration positions wheels in an "X" pattern. Forward/backward travel slightly exceeds other directions due to friction characteristics.

## Control Equations — Robot-Centric

```java
double y = -gamepad1.left_stick_y;    // forward/backward (negated: up = positive)
double x = gamepad1.left_stick_x * 1.1; // strafe (1.1 corrects mechanical imperfection)
double rx = gamepad1.right_stick_x;   // rotation

double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
double frontLeftPower  = (y + x + rx) / denominator;
double backLeftPower   = (y - x + rx) / denominator;
double frontRightPower = (y - x - rx) / denominator;
double backRightPower  = (y + x - rx) / denominator;
```

**Denominator normalization** maintains consistent power ratios — without it, values exceeding 1 get clipped and distort the intended movement direction.

## Motor Direction

Most FTC motors spin counterclockwise with positive power. Typically, reverse right-side motors. If the robot moves backward when commanded forward, reverse left-side instead.

This robot's configuration:
- `frontleft` / `backleft`: REVERSE
- `frontright` / `backright`: FORWARD

## Field-Centric Driving

Field-centric mode uses the IMU to rotate joystick inputs, allowing movement relative to field position rather than robot orientation:

```java
double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

double frontLeftPower  = (rotY + rotX + rx) / denominator;
double backLeftPower   = (rotY - rotX + rx) / denominator;
double frontRightPower = (rotY - rotX - rx) / denominator;
double backRightPower  = (rotY + rotX - rx) / denominator;
```

## Notes for This Robot

- Characterized max velocities: forward 54.6 in/s, lateral 74.5 in/s
- Pedro Pathing handles autonomous path following — these equations apply to TeleOp
- `forwardZeroPowerAcceleration`: −30.34; `lateralZeroPowerAcceleration`: −60.96
