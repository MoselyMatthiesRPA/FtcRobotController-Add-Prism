# FTC SDK Overview — DECODE Season 2025–2026

Source: https://github.com/FIRST-Tech-Challenge/FtcRobotController

## Overview

The FTC Robot Controller SDK is the source for the Android app that controls FTC competition robots. It targets the DECODE season (2025–2026).

## Requirements

- Android Studio Ladybug (2024.2) or later
- Blocks and OnBot Java do not require Android Studio

## Key Resources

- Official docs: https://ftc-docs.firstinspires.org
- Javadoc: https://javadoc.io (search for FTC SDK)
- Community forum: FTC Technology forum

## SDK Version 11.1 Highlights

- Gamepad trigger boolean access with edge detection
- GoBilda Pinpoint v2 support
- New webcam calibrations

## Sample OpModes

Located at `robotcontroller/external/samples/` — useful references for:
- `SensorLimelight3A.java` — Limelight3A initialization and result reading
- Mecanum drive examples
- IMU usage

## Key APIs for This Robot

### HardwareMap
```java
// Motor
DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "motorName");

// Servo
Servo servo = hardwareMap.get(Servo.class, "servoName");

// Limelight
Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

// GoBilda Pinpoint
GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
```

### DcMotorEx Key Methods
```java
motor.setDirection(DcMotorSimple.Direction.REVERSE);
motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
motor.setVelocity(ticksPerSecond);
double vel = motor.getVelocity();
```

### Bulk Caching (Performance)
```java
List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
for (LynxModule hub : hubs) {
    hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
}
```

## Motor/Servo Defaults at OpMode Start

- All motors default to Direction.FORWARD
- All servos default to Direction.FORWARD
- Always set directions explicitly in init
