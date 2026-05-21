package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.lynx.LynxModule;
import java.util.List;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import androidx.annotation.Nullable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;


@Configurable
@TeleOp(name = "turret test", group = "Competition")

public class servo extends OpMode {

    // Robot pose (updated from Pedro)
    private double robotX = 0;
    private double robotY = 0;
    private double robotHeading = (3.14/2.0); // radians
    DcMotorEx intake;
    Turret turret;
    DcMotor frontLeft, frontRight, backLeft, backRight;
    DualPidMotor flywheel;
    Servo rbstop, rhoodtilt;
    Limelight3A limelight;
    GoBildaPrismDriver prism;
    PrismAnimations.Solid solidGreen = new PrismAnimations.Solid(Color.GREEN);
    PrismAnimations.Solid solidPink = new PrismAnimations.Solid(Color.PINK);
    PrismAnimations.Solid solidRed = new PrismAnimations.Solid(Color.RED);
    public double targetAngle;
    public double turretAngle;
    public double output;

    // --- Compute magnitude ---
    public double deadband = 0.2;
    public static double kP = 0.01;
    public static double kD = 0.0007;
    public static double kF = 0.01;
    public static double MAX_ANGLE = 120;
    public static double MIN_ANGLE = -90;
    public static double maxPower = 1;
    public static double goaltarget = 0;
    public double lastOutput = 0;
    public static double maxChange = 0.06;
    public double lastGoodtTx = 0;
    private static final double MIN_RPM = 0.0;
    private static final double MAX_RPM = 5800.0;

    private static double MAX_TILT =  0.65;
    private static final double MIN_TILT = 0.02;

    private static final double INCHES_PER_METER = 39.3701;

    // --- YOUR CAL POINTS ---
    private static final double RAW_AT_1M_IN = 40.0;
    private static final double RAW_AT_2M_IN = 76.5;

    private static final double TRUE_1M_IN = INCHES_PER_METER;
    private static final double TRUE_2M_IN = 2.0 * INCHES_PER_METER;

    // Linear correction: corrected = A*raw + B, fit to (RAW_AT_1M -> TRUE_1M) and (RAW_AT_2M -> TRUE_2M)
    private static final double DIST_A = (TRUE_2M_IN - TRUE_1M_IN) / (RAW_AT_2M_IN - RAW_AT_1M_IN);
    private static final double DIST_B = TRUE_1M_IN - (DIST_A * RAW_AT_1M_IN);
    public static double RPM_AT_1M = 2125;
    public static double RPM_AT_2M = 2500;
    public static double RPM_AT_FAR = 3000;
    public static double TILT_AT_FAR = 0.7;
    public static double TILT_AT_1M = .25;
    public static double TILT_AT_2M = .6;
    public static double RPM_M, RPM_C, TILT_M, TILT_C;
    long lastTagTime = 0;
    public static double TAG_TIMEOUT_MS = 1000;
    public double targetFlywheelRPM = 0;
    public double lastGoodFlywheelRPM;
    public double lastGoodHoodTilt = 0;
    double error;
    public double baseTarget = 0;
    public static double targetTurretangle = 2; // target angle between the turret and the target in degrees
    public double usedRPM;
    public double usedTILT;
    public static double intakeIntakingTargetRPM = 900;
    public static double intakeShootingTargetRPM = 600;
    public double intakingRPM;
    public static double intakeFarRPM = 250;
    public static double intakeCloseRPM = 500;
    public static double stopperDown = 0.13;
    public static double maxchangescaler = 10;
    public double lastGoodIntakeRPM;
    public static double stopperUp = 0;
    public double distanceInches;
    public double lastError;
    public double lastBaseTarget;
    public static double turretAcceptableError = 0.5;
    ElapsedTime loopTimer = new ElapsedTime();
    public static double MAX_TAG_AMBIGUITY = 0.7;
    public static double MAX_TAG_DISTANCE_M = 4.0;
    // Tune this: minimum displacement (inches) to accept a vision update
// Prevents jitter when the robot is stationary
    public static double MIN_POSE_JUMP_INCHES = 0.5;
    public static double MAX_POSE_JUMP_INCHES = 24.0; // reject wild outliers
    public static double LL_HEADING_OFFSET = 90.0; // degrees — tune in Step 2
    // Reject LL frames captured before the most recent updateRobotOrientation
    // had time to take effect; otherwise MT2 may return a pose solved with a
    // stale yaw. ~100 ms is comfortably longer than one Limelight frame.
    public static double MAX_LL_STALENESS_MS = 100.0;


    private void updatePoseFromLimelight() {
        double limelightYaw = LimelightHeading.pedroHeadingToLimelightDeg(robotHeading);
        limelight.updateRobotOrientation(limelightYaw);

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            telemetry.addData("LL Status", "No valid result");
            return;
        }

        double staleness = result.getStaleness();
        telemetry.addData("LL Staleness ms", staleness);
        if (staleness > MAX_LL_STALENESS_MS) {
            telemetry.addData("LL Rejected", "Stale frame (" + staleness + " ms)");
            return;
        }

        Pose3D botpose = result.getBotpose_MT2();
        if (botpose == null) {
            telemetry.addData("LL Status", "No botpose");
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            telemetry.addData("LL Status", "No fiducials");
            return;
        }

        // Delegate the LL (FTC-standard, center origin, meters) -> Pedro
        // (bottom-left origin, inches) conversion to Pedro's PoseConverter so
        // we don't hand-roll signs/axes for DECODE. The Pedro docs explicitly
        // pair MT2 botpose with InvertedFTCCoordinates for this field.
        Pose2D ftcPose2D = new Pose2D(
                DistanceUnit.INCH,
                botpose.getPosition().toUnit(DistanceUnit.INCH).x,
                botpose.getPosition().toUnit(DistanceUnit.INCH).y,
                AngleUnit.RADIANS,
                Math.toRadians(botpose.getOrientation().getYaw(AngleUnit.DEGREES)));
        Pose ftcStandard = PoseConverter.pose2DToPose(ftcPose2D, InvertedFTCCoordinates.INSTANCE);
        Pose llPedroPose = ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        double pedroX = llPedroPose.getX();
        double pedroY = llPedroPose.getY();

        // Log before filters
        telemetry.addData("LL Pedro X", pedroX);
        telemetry.addData("LL Pedro Y", pedroY);
        telemetry.addData("Pedro X", robotX);
        telemetry.addData("Pedro Y", robotY);
        telemetry.addData("Tags Seen", fiducials.size());
        telemetry.addData("LL Raw X", botpose.getPosition().x);
        telemetry.addData("LL Raw Y ", botpose.getPosition().y);
        telemetry.addData("LL Yaw sent", LimelightHeading.pedroHeadingToLimelightDeg(robotHeading));
        telemetry.addData("RAW X m", botpose.getPosition().x);
        telemetry.addData("RAW Y m", botpose.getPosition().y);
        telemetry.addData("RAW Z m", botpose.getPosition().z);
        telemetry.addData("RAW Yaw", botpose.getOrientation().getYaw(AngleUnit.DEGREES));


        // Distance filter
        for (LLResultTypes.FiducialResult tag : fiducials) {
            Pose3D robotRelTag = tag.getRobotPoseTargetSpace();
            if (robotRelTag == null) { telemetry.addData("LL Rejected", "Null tag pose"); return; }
            double tx = robotRelTag.getPosition().x;
            double ty = robotRelTag.getPosition().y;
            double tz = robotRelTag.getPosition().z;
            if (Math.sqrt(tx*tx + ty*ty + tz*tz) > MAX_TAG_DISTANCE_M) {
                telemetry.addData("LL Rejected", "Tag too far");
                return;
            }
        }

        // Jump filter
        double jumpDist = Math.hypot(pedroX - robotX, pedroY - robotY);
        telemetry.addData("LL Jump Dist", jumpDist);
        if (jumpDist > MAX_POSE_JUMP_INCHES) {
            telemetry.addData("LL Rejected", "Jump too large: " + jumpDist);
            return;
        } else {

        telemetry.addData("LL Status", "ACCEPTED");

         follower.setPose(new Pose(pedroX, pedroY, robotHeading));
            }
    }

    public static void updateModels() {
        // RPM model: RPM = M*Distance(in) + C, using (TRUE_1M, RPM_AT_1M) and (TRUE_2M, RPM_AT_2M)
        RPM_M = (RPM_AT_2M - RPM_AT_1M) / (TRUE_2M_IN - TRUE_1M_IN);
        RPM_C = RPM_AT_1M - (RPM_M * TRUE_1M_IN);

        TILT_M = (TILT_AT_2M - TILT_AT_1M) / (TRUE_2M_IN - TRUE_1M_IN);
        TILT_C = TILT_AT_1M - (TILT_M * TRUE_1M_IN);
    }
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
    private double calculateTurretAngleFromOdometry() {
        // Get vector from robot to goal (field coordinates)
        double deltaX = FieldPositions.targetGoalX - robotX;
        double deltaY = FieldPositions.targetGoalY - robotY;

        // Calculate field-centric angle to goal (in radians)
        double fieldCentricAngle = Math.atan2(deltaY, deltaX);

        // Convert to robot-centric by subtracting robot's heading
        double robotCentricAngle = -(fieldCentricAngle - robotHeading);

        // Convert to degrees
        double angleDegrees = Math.toDegrees(robotCentricAngle);

        // Normalize to [-180, 180] range
        while (angleDegrees > 180) angleDegrees -= 360;
        while (angleDegrees < -180) angleDegrees += 360;

        return angleDegrees+targetTurretangle;
    }
    public static class FieldPositions {
        // Define goal positions in inches (Pedro's coordinate system)
        // Adjust these based on your field's coordinate system
        public static final double RED_GOAL_X = 130.3727;
        public static final double RED_GOAL_Y = 127.6425;

        public static final double BLUE_GOAL_X = 13.6273;
        public static final double BLUE_GOAL_Y = 127.6425;

        // Set based on alliance
        public static double targetGoalX = BLUE_GOAL_X;
        public static double targetGoalY = BLUE_GOAL_Y;
    }
    private double calculateDistanceToGoal() {
        // Get vector from robot to goal
        double deltaX = FieldPositions.targetGoalX - robotX;
        double deltaY = FieldPositions.targetGoalY - robotY;

        // Pythagorean theorem
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }
    private void updatePoseFromPedro() {
        Pose currentPose = follower.getPose();
        robotX = currentPose.getX(); // Left is Positive
        robotY = currentPose.getY(); // Forward is Positive
        robotHeading = currentPose.getHeading(); // In Radians
    }
    boolean override;
    boolean taglastseen;
    boolean wasspinningup;
    public double targetHoodTilt = 0;
    private Follower follower;
    public static Pose startingPose;
    private TelemetryManager telemetryM;


    @Override
    public void init(){
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        frontRight = hardwareMap.get(DcMotor.class, "frontright");
        frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        backRight = hardwareMap.get(DcMotor.class, "backright");
        backLeft = hardwareMap.get(DcMotor.class, "backleft");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rbstop = hardwareMap.get(Servo.class, "rbstop");
        rhoodtilt = hardwareMap.get(Servo.class, "rhoodtilt");
        rhoodtilt.setDirection(Servo.Direction.REVERSE);
        flywheel = new DualPidMotor (hardwareMap, "bottomflywheel", "topflywheel");
        turret = new Turret(hardwareMap, true);


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose(72,24,(Math.toRadians(90))) : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        // Seed the yaw before the first frame is solved; otherwise MT2 may
        // compute a botpose with the default yaw=0 and we'd see a 180-deg
        // mirrored position error on the very first reading.
        limelight.updateRobotOrientation(
                LimelightHeading.pedroHeadingToLimelightDeg(follower.getPose().getHeading()));

        rbstop.setPosition(0);
        rhoodtilt.setPosition(MIN_TILT);

        override = false;

        List<LynxModule> allHubs;
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    @Override
    public void start(){
        double limelightYaw = LimelightHeading.pedroHeadingToLimelightDeg(follower.getPose().getHeading());
        limelight.updateRobotOrientation(limelightYaw);
    }

    @Override
    public void loop() {
        follower.update();
        updatePoseFromPedro();
        updatePoseFromLimelight();
//        flywheel.setVelocity(targetFlywheelRPM);
        telemetryM.update();

        if (distanceInches < 105)    {
            // Clamps outputs to motors and servos to the usable pre-set range from our variables before init, and calculates the output RPM and Tilt
            targetFlywheelRPM = clamp((RPM_M * distanceInches) + RPM_C, MIN_RPM, MAX_RPM);
            targetHoodTilt = clamp((TILT_M * distanceInches) + TILT_C, MIN_TILT, MAX_TILT);
            lastGoodHoodTilt = targetHoodTilt;
            lastGoodFlywheelRPM = targetFlywheelRPM;
            intakingRPM = intakeCloseRPM;
        } else {
            targetFlywheelRPM = RPM_AT_FAR;
            targetHoodTilt = TILT_AT_FAR;
            intakingRPM = intakeFarRPM;
        }

        telemetryM.debug("position", follower.getPose());
        double dt = loopTimer.seconds();
        loopTimer.reset();

        double cx = gamepad2.right_stick_x;
        double cy = -gamepad2.right_stick_y;
        double magnitude = Math.hypot(cx, cy); // magnitude is basically a measurement of how much we are pushing the joystick, as in the hypotenuse of our joystick vector. Not just
        // in one direction. A magnitude of 1 is the stick being pushed exactly forward/backwards or right/left.

        boolean turretjoystick = ((magnitude > 0.5));

        turretAngle = turret.getCurrentAngle(); // determines our Turret position in degrees from 0(0 is set at initiation, needs to be set exactly forwards or our limits wont work)


            if (turretjoystick) {
                // Manual joystick control
                baseTarget = Math.toDegrees(Math.atan2(cx, cy));
            } else {
                // Odometry-based targeting
                baseTarget = (calculateTurretAngleFromOdometry());

                // Calculate distance for your existing shooter calculations
                distanceInches = calculateDistanceToGoal();

            }
            lastBaseTarget = baseTarget;

        if (gamepad2.right_trigger > 0.1){
            rbstop.setPosition(0.3);
            intake.setVelocity((145.1*intakingRPM)/60);
        } else if (gamepad2.left_trigger > 0.1 && gamepad2.right_trigger < 0.1){
            rhoodtilt.setPosition(0);
            rhoodtilt.setPosition(0);
            intake.setVelocity((145.1*intakeIntakingTargetRPM)/60);
        } else{
            intake.setVelocity(0);
            rbstop.setPosition(0);
            rhoodtilt.setPosition(targetHoodTilt);
        }

// Reads joystick values for our Mecanum logic
        double rx = gamepad1.right_stick_x;
        double y = -gamepad1.left_stick_y;  // Forward is positive
        double x = gamepad1.left_stick_x;  // Strafe

// Mecanum mixing
        double frontLeftPower = y + x + (0.8 * rx);
        double backLeftPower = y - x + (0.8 * rx);
        double frontRightPower = y - x - (0.8 * rx);
        double backRightPower = y + x - (0.8 * rx);

// Normalize powers so no value exceeds 1.0
        double max = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                Math.max(Math.abs(frontRightPower), Math.abs(backRightPower)));

        if (max > 1.0) {
            frontLeftPower /= max;
            backLeftPower /= max;
            frontRightPower /= max;
            backRightPower /= max;
        }

// Send power to motors
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);


        telemetry.addData("current angle", turretAngle);
        telemetry.addData("target angle", turret.getTargetAngle());
        telemetry.addData("output", turret.getOutput());
        telemetry.addData("override", override);
        telemetry.addData("target RPM", targetFlywheelRPM);
        telemetry.addData("current flywheel RPM", flywheel.getCurrentRPM());
        telemetry.addData("targethoodtilt", lastGoodHoodTilt);
        telemetry.addData("distance from tag", distanceInches);
        telemetry.addData("Loop Time (ms)", dt * 1000);
        telemetry.addData("Pedropathing X", robotX);
        telemetry.addData("Pedropathing Y", robotY);
        telemetry.addData("Pedro Heading", Math.toDegrees(robotHeading));

        telemetry.update();

        turret.setTargetAngle(baseTarget);
        turret.update(dt);
        flywheel.Update();
        updateModels();

    }
}