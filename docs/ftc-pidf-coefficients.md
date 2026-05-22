# Changing PIDF Coefficients in FTC

Source: https://ftc-docs.firstinspires.org/programming_resources/shared/pidf_coefficients/pidf-coefficients.html

## Overview

The REV Robotics Control Hub or Expansion Hub enables users to adjust PIDF coefficients for closed-loop motor control. These coefficients are specific to each motor port and RunMode configuration.

## Key Points

**Persistence Note:** Changes made to the PIDF coefficients do NOT persist if you power cycle the REV Robotics Control Hub or REV Robotics Expansion Hub. To retain modifications, store state information on the Control Hub or Android device using Android's data storage capabilities. **Always set PIDF coefficients during OpMode init.**

## Two Implementation Approaches

### Method 1: Using DcMotorEx Class

```java
DcMotorEx motor = (DcMotorEx) hardwareMap.get(DcMotor.class, "motorName");
PIDFCoefficients pidf = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
pidf.p = 2.5;
pidf.i = 0.1;
pidf.d = 0.2;
pidf.f = 0.5;
motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
```

- Requires casting the motor to DcMotorEx type
- Calls `getPIDFCoefficients` to retrieve current values
- Creates/modifies a PIDFCoefficients object with desired P, I, D, F values
- Applies changes via `setPIDFCoefficients`
- Coefficients must be tuned per motor based on its planned usage

### Method 2: Using DcMotorControllerEx Class

- Retrieves the motor controller via `getController()`
- Obtains the motor's port number using `getPortNumber()`
- Applies PIDF adjustments at the controller level rather than motor level

## Deprecation Note

SDK 7.0 maintains backward compatibility with older PID-only methods, though these are now deprecated in favor of PIDF implementations.

## Notes for This Robot

- Flywheel uses DualPidMotor wrapper with P=120.0, D=0.0, F=kf×RPM (kf≈0.00380)
- Turret uses software PD loop (RUN_WITHOUT_ENCODER), not the SDK PIDF system
- Both must be initialized each OpMode start — PIDF does not persist across power cycles
