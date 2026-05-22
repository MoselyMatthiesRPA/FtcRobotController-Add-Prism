# Limelight MegaTag2 Robot Localization

Source: https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2

## Overview

MegaTag2 is an AprilTag-based localization system that provides precise, ambiguity-free robot positioning. It addresses MegaTag1 limitations by eliminating pose ambiguity and improving robustness against noise and tag placement inaccuracies. Works from a single tag regardless of viewing angle.

## Core Requirements

- Robot-space camera pose configured in webUI or API
- Uploaded field map (.fmap file)
- Frame-by-frame calls to `updateRobotOrientation()` with current yaw
- Coordinate system origin understanding (blue corner for 2024+)

## FTC Implementation (our robot)

```java
// Each loop iteration, before getLatestResult():
limelight.updateRobotOrientation(currentYawDegrees);

LLResult result = limelight.getLatestResult();
if (result != null && result.isValid()) {
    Pose3D botpose = result.getBotpose_MT2();
    // x, y are field coordinates; yaw is field-relative heading
}
```

## Filtering Unreliable Data

Reject updates when the robot is spinning fast (angular velocity threshold):

```java
if (Math.abs(imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate) < 180) {
    // safe to use MT2 pose
}
```

## Tag ID Filtering

Restrict pose calculation to specific tag IDs:

```java
int[] validIDs = {3, 4};
LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs);
```

## Limelight 4 IMU Modes (for reference)

| Mode | Source | Purpose |
|------|--------|---------|
| 0 | External only | No internal processing |
| 1 | External seed | Calibrate internal offset each frame |
| 2 | Internal only | Uses built-in IMU exclusively |
| 3 | Internal + MT1 | Complementary filter with vision correction |
| 4 | Internal + External | Recommended — combines both sources |

## Key Advantages over MegaTag1

- Single-tag capable (robust even with partial field coverage)
- Eliminates pose flip ambiguity at low tag angles
- Tolerant of image noise and physical tag placement inaccuracies
- Requires only yaw input (pitch/roll optional for enhanced accuracy)

## Notes for This Robot

- Pipeline 2 is the MegaTag2 pipeline
- Heading fed into `updateRobotOrientation()` must be in FTC convention (0° = audience)
- Pedro convention differs by −90°; see `LimelightHeading.java`
- Staleness check recommended before injecting pose into Pedro follower
