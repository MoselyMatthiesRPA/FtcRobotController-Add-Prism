package org.firstinspires.ftc.teamcode.lawrence.vision;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.lawrence.constants.RobotConstants;

import java.util.List;

/**
 * Fuses Limelight MegaTag2 AprilTag pose into the Pedro Pathing follower.
 *
 * Coordinate mapping: Limelight botpose is FTC standard (meters, center origin,
 * +X toward audience, +Y red→blue). Pedro uses inches, bottom-left origin, axes
 * rotated 90° from FTC: Pedro +X = FTC +Y, Pedro +Y = FTC -X.
 * So: pedroX = ftcY_in + 72,  pedroY = -ftcX_in + 72
 */
public class LimelightLocalizer {

    private final Limelight3A limelight;
    private final Follower follower;

    // Debug fields — read by DebugDrive telemetry each loop
    public String lastFrameStatus    = "NULL";
    public double lastStalenessMs    = 0;
    public int    lastTagCount       = 0;
    public double lastJumpIn         = 0;
    public int    consecutiveRejects = 0;
    public double lastAcceptedX      = 0;
    public double lastAcceptedY      = 0;
    public double lastFtcX           = 0;
    public double lastFtcY           = 0;

    public LimelightLocalizer(Limelight3A limelight, Follower follower) {
        this.limelight = limelight;
        this.follower  = follower;
    }

    /**
     * Runs one pose-fusion cycle. Call every loop iteration.
     * @param pedroHeadingRad current robot heading from Pedro (radians)
     * @return true if a pose update was accepted and injected
     */
    public boolean update(double pedroHeadingRad) {
        double limelightYaw = HeadingConverter.pedroToLimelightDeg(pedroHeadingRad);
        limelight.updateRobotOrientation(limelightYaw);

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            lastFrameStatus = "NULL";
            consecutiveRejects++;
            return false;
        }

        lastStalenessMs = result.getStaleness();
        if (lastStalenessMs > RobotConstants.MAX_LL_STALENESS_MS) {
            lastFrameStatus = "STALE";
            consecutiveRejects++;
            return false;
        }

        Pose3D botpose = result.getBotpose_MT2();
        if (botpose == null) {
            lastFrameStatus = "NULL";
            consecutiveRejects++;
            return false;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            lastFrameStatus = "NULL";
            consecutiveRejects++;
            return false;
        }
        lastTagCount = fiducials.size();

        for (LLResultTypes.FiducialResult tag : fiducials) {
            Pose3D tagPose = tag.getRobotPoseTargetSpace();
            if (tagPose == null) {
                lastFrameStatus = "NULL";
                consecutiveRejects++;
                return false;
            }
            double tx = tagPose.getPosition().x;
            double ty = tagPose.getPosition().y;
            double tz = tagPose.getPosition().z;
            if (Math.sqrt(tx * tx + ty * ty + tz * tz) > RobotConstants.MAX_TAG_DISTANCE_M) {
                lastFrameStatus = "REJECTED_TAG_DIST";
                consecutiveRejects++;
                return false;
            }
        }

        double ftcX = botpose.getPosition().toUnit(DistanceUnit.INCH).x;
        double ftcY = botpose.getPosition().toUnit(DistanceUnit.INCH).y;
        lastFtcX = ftcX;
        lastFtcY = ftcY;

        double pedroX = ftcY + 72.0;
        double pedroY = -ftcX + 72.0;

        Pose current = follower.getPose();
        double jump = Math.hypot(pedroX - current.getX(), pedroY - current.getY());
        lastJumpIn = jump;

        if (jump < RobotConstants.MIN_POSE_JUMP_IN) {
            lastFrameStatus = "REJECTED_JITTER";
            consecutiveRejects++;
            return false;
        }
        if (jump > RobotConstants.MAX_POSE_JUMP_IN) {
            lastFrameStatus = "REJECTED_OUTLIER";
            consecutiveRejects++;
            return false;
        }

        follower.setPose(new Pose(pedroX, pedroY, pedroHeadingRad));
        lastFrameStatus    = "ACCEPTED";
        lastAcceptedX      = pedroX;
        lastAcceptedY      = pedroY;
        consecutiveRejects = 0;
        return true;
    }
}
