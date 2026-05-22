# Limelight 3A — FTC Java Programming Guide

Source: https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming

## Core Initialization

```java
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
limelight.setPollRateHz(100);  // must be called before start()
limelight.start();
limelight.pipelineSwitch(2);   // switch to MegaTag2 pipeline
```

**Important:** `setPollRateHz()` must be called before `start()`. Poll rate is clamped 1–250 Hz.

## Pipeline Management

Up to 10 distinct pipelines can be stored. Switching is instantaneous:

```java
limelight.pipelineSwitch(0);  // switch to pipeline 0
int index = result.getPipelineIndex();  // check current pipeline
```

## Results Retrieval

```java
LLResult result = limelight.getLatestResult();
if (result != null && result.isValid()) {
    double tx = result.getTx();  // horizontal offset in degrees
    double ty = result.getTy();  // vertical offset in degrees
    double ta = result.getTa();  // target area (0–100%)
}
```

## MegaTag2 Localization (FTC API)

```java
// Call every loop before getLatestResult()
limelight.updateRobotOrientation(currentHeadingDegrees);

LLResult result = limelight.getLatestResult();
if (result != null && result.isValid()) {
    Pose3D botpose = result.getBotpose_MT2();
    double x = botpose.getPosition().x;
    double y = botpose.getPosition().y;
    double yaw = botpose.getOrientation().getYaw(AngleUnit.DEGREES);
}
```

Coordinate system: origin (0,0,0) at field center.

## Data Freshness

```java
long staleness = result.getStaleness();  // milliseconds since capture
if (staleness < 100) { /* recent enough to use */ }
```

## AprilTag / Fiducial Results

```java
List<FiducialResult> fiducials = result.getFiducialResults();
for (FiducialResult fiducial : fiducials) {
    int id = fiducial.getFiducialId();
    Pose3D pose = fiducial.getRobotPoseTargetSpace();
}
```

## Notes for This Robot

- Hardware name: `"limelight"`, type: `Limelight3A`
- Poll rate: 100 Hz
- Default pipeline: 2 (MegaTag2 AprilTag localization)
- FTC heading 0° = audience side; Pedro 0° = red alliance goal; offset = +90°
- Reject stale frames (staleness check) before fusing pose into Pedro
- See `LimelightHeading.java` for heading conversion utility
