# Pedro Pathing — Constants

Source: https://pedropathing.com/docs/pathing/constants

## Overview

The Constants file is the central configuration hub for Pedro Pathing. It contains four types of constants:

1. **Follower Constants** — derived from automatic, PID, and centripetal tuners
2. **Drivetrain Constants** — robot-specific motor names and directions
3. **Localizer Constants** — hardware-specific configuration (sensor names, offsets)
4. **Path Constraints** — conditions determining when a path may conclude

## Basic Structure

```java
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants();

    public static PathConstraints pathConstraints =
        new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .build();
    }
}
```

## This Robot's Tuned Values (Constants.java)

### Drivetrain Motor Names
| Constant | Value |
|---|---|
| leftFrontMotorName | `"frontleft"` |
| rightFrontMotorName | `"frontright"` |
| leftRearMotorName | `"backleft"` |
| rightRearMotorName | `"backright"` |
| leftFrontMotorDirection | REVERSE |
| rightFrontMotorDirection | FORWARD |
| leftRearMotorDirection | REVERSE |
| rightRearMotorDirection | FORWARD |

### Follower Constants
| Constant | Value |
|---|---|
| translationalPIDFCoefficients | (0.1, 0, 0.001, 0.02) |
| headingPIDFCoefficients | (1, 0.02, 0.0001, 0.03) |
| drivePIDFCoefficients | (0.01, 0, 0, 0, 0.02) |
| centripetalScaling | 0.005 |
| mass | 10 kg |
| maxPower | 1.0 |
| forwardZeroPowerAcceleration | −30.335 |
| lateralZeroPowerAcceleration | −60.965 |
| yVelocity (max forward) | 54.56 in/s |
| xVelocity (max lateral) | 74.46 in/s |

### Localizer Constants (Pinpoint)
| Constant | Value |
|---|---|
| hardwareName | `"odo"` |
| forwardPodY | 0 mm |
| strafePodX | −120 mm |
| encoderResolution | goBILDA_4_BAR_POD |
| forwardEncoderDirection | FORWARD |
| strafeEncoderDirection | REVERSED |

### Path Constraints
| Constant | Value |
|---|---|
| maxVelocity | 0.99 |
| maxAcceleration | 100 |
| maxAngularVelocity | 0.55 rad/s |
| maxAngularAcceleration | 0.5 rad/s² |

## Tuning Pages

Consult the Pedro Pathing tuning guides to re-tune:
- Localization test: forward should increase x, strafe-left should increase y
- Pinpoint: avoid I²C port 0
