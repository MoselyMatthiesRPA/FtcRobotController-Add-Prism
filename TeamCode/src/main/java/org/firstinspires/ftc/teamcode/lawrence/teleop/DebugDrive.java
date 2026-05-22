package org.firstinspires.ftc.teamcode.lawrence.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.lawrence.constants.FieldPositions;
import org.firstinspires.ftc.teamcode.lawrence.constants.HardwareNames;
import org.firstinspires.ftc.teamcode.lawrence.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.lawrence.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.lawrence.subsystems.Turret;
import org.firstinspires.ftc.teamcode.lawrence.vision.HeadingConverter;
import org.firstinspires.ftc.teamcode.lawrence.vision.LimelightLocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "Debug Drive", group = "Lawrence")
public class DebugDrive extends OpMode {

    public static double RPM_AT_SPEED_THRESHOLD = 150.0;
    public static double JOYSTICK_DEADZONE      = 0.5;
    public static double ROTATION_SCALE         = 0.8;
    public static double turretAcceptableError  = 0.5;

    private DcMotorEx intake;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private Flywheel flywheel;
    private Turret turret;
    private Servo stopper, hood;
    private Limelight3A limelight;
    private GoBildaPrismDriver prism;
    private VoltageSensor battSensor;

    private Follower follower;
    private LimelightLocalizer localizer;
    private TelemetryManager telemetryM;

    private final PrismAnimations.Solid ledGreen = new PrismAnimations.Solid(Color.GREEN);
    private final PrismAnimations.Solid ledPink  = new PrismAnimations.Solid(Color.PINK);
    private final PrismAnimations.Solid ledRed   = new PrismAnimations.Solid(Color.RED);

    private double robotX       = 0;
    private double robotY       = 0;
    private double robotHeading = Math.PI / 2.0;

    private double targetFlywheelRPM = 0;
    private double targetHoodTilt    = RobotConstants.MIN_TILT;
    private double distanceInches    = 0;
    private double intakingRPM       = 0;

    private static double RPM_M, RPM_C, TILT_M, TILT_C;

    private double baseTarget = 0;
    private boolean allianceRed = false;
    private boolean prevYBtn    = false;

    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void init() {
        frontLeft  = hardwareMap.get(DcMotor.class, HardwareNames.FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotor.class, HardwareNames.FRONT_RIGHT);
        backLeft   = hardwareMap.get(DcMotor.class, HardwareNames.BACK_LEFT);
        backRight  = hardwareMap.get(DcMotor.class, HardwareNames.BACK_RIGHT);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotorEx.class, HardwareNames.INTAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        stopper = hardwareMap.get(Servo.class, HardwareNames.STOPPER_RIGHT);
        hood    = hardwareMap.get(Servo.class, HardwareNames.HOOD_RIGHT);
        hood.setDirection(Servo.Direction.REVERSE);

        flywheel = new Flywheel(hardwareMap);
        turret   = new Turret(hardwareMap, true);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 24, Math.toRadians(90)));
        follower.update();

        limelight = hardwareMap.get(Limelight3A.class, HardwareNames.LIMELIGHT);
        limelight.pipelineSwitch(0);
        limelight.start();
        limelight.updateRobotOrientation(
                HeadingConverter.pedroToLimelightDeg(follower.getPose().getHeading()));
        localizer = new LimelightLocalizer(limelight, follower);

        battSensor = hardwareMap.voltageSensor.iterator().next();

        prism = hardwareMap.get(GoBildaPrismDriver.class, HardwareNames.PRISM);
        configureLed(ledGreen);
        configureLed(ledPink);
        configureLed(ledRed);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, ledGreen);

        stopper.setPosition(RobotConstants.STOPPER_UP);
        hood.setPosition(RobotConstants.MIN_TILT);

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        updateModels();
    }

    @Override
    public void start() {
        limelight.updateRobotOrientation(
                HeadingConverter.pedroToLimelightDeg(follower.getPose().getHeading()));
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        // Alliance toggle — gamepad1 Y (rising edge)
        boolean yNow = gamepad1.y;
        if (yNow && !prevYBtn) {
            allianceRed = !allianceRed;
            FieldPositions.targetGoalX = allianceRed ? FieldPositions.RED_GOAL_X : FieldPositions.BLUE_GOAL_X;
            FieldPositions.targetGoalY = allianceRed ? FieldPositions.RED_GOAL_Y : FieldPositions.BLUE_GOAL_Y;
        }
        prevYBtn = yNow;

        // Odometry
        follower.update();
        Pose pose = follower.getPose();
        robotX       = pose.getX();
        robotY       = pose.getY();
        robotHeading = pose.getHeading();

        // Vision pose fusion
        localizer.update(robotHeading);

        // Distance & shooter model
        distanceInches = calculateDistanceToGoal();
        double correctedDist = RobotConstants.DIST_A * distanceInches + RobotConstants.DIST_B;

        String shootRegime;
        if (correctedDist < RobotConstants.FAR_THRESHOLD_IN) {
            targetFlywheelRPM = clamp(RPM_M * correctedDist + RPM_C,
                    RobotConstants.MIN_RPM, RobotConstants.MAX_RPM);
            targetHoodTilt = clamp(TILT_M * correctedDist + TILT_C,
                    RobotConstants.MIN_TILT, RobotConstants.MAX_TILT);
            intakingRPM = RobotConstants.CLOSE_RPM;
            shootRegime = "CLOSE";
        } else {
            targetFlywheelRPM = RobotConstants.RPM_AT_FAR;
            targetHoodTilt    = RobotConstants.TILT_AT_FAR;
            intakingRPM       = RobotConstants.FAR_RPM;
            shootRegime       = "FAR";
        }

        // Model predictions at corrected dist (shown regardless of regime for comparison)
        double modelRpm  = clamp(RPM_M * correctedDist + RPM_C, RobotConstants.MIN_RPM, RobotConstants.MAX_RPM);
        double modelTilt = clamp(TILT_M * correctedDist + TILT_C, RobotConstants.MIN_TILT, RobotConstants.MAX_TILT);

        // Turret targeting
        double cx = gamepad2.right_stick_x;
        double cy = -gamepad2.right_stick_y;
        boolean turretJoystick = Math.hypot(cx, cy) > JOYSTICK_DEADZONE;
        if (turretJoystick) {
            baseTarget = Math.toDegrees(Math.atan2(cx, cy));
        } else {
            baseTarget = calculateTurretAngle();
        }

        // Goal geometry for display
        double goalDX        = FieldPositions.targetGoalX - robotX;
        double goalDY        = FieldPositions.targetGoalY - robotY;
        double fieldAngleDeg = Math.toDegrees(Math.atan2(goalDY, goalDX));
        double robotAngleDeg = Math.toDegrees(-(Math.atan2(goalDY, goalDX) - robotHeading));
        while (robotAngleDeg >  180) robotAngleDeg -= 360;
        while (robotAngleDeg < -180) robotAngleDeg += 360;

        // Intake & servo control
        String intakeState;
        if (gamepad2.right_trigger > 0.1) {
            stopper.setPosition(RobotConstants.STOPPER_DOWN);
            intake.setVelocity(ticksPerSec(intakingRPM));
            intakeState = "FEEDING";
        } else if (gamepad2.left_trigger > 0.1) {
            hood.setPosition(RobotConstants.MIN_TILT);
            intake.setVelocity(ticksPerSec(RobotConstants.INTAKING_RPM));
            intakeState = "INTAKING";
        } else {
            intake.setVelocity(0);
            stopper.setPosition(RobotConstants.STOPPER_UP);
            hood.setPosition(targetHoodTilt);
            intakeState = "OFF";
        }

        // Mecanum drive
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;
        double flPower = y + x + ROTATION_SCALE * rx;
        double blPower = y - x + ROTATION_SCALE * rx;
        double frPower = y - x - ROTATION_SCALE * rx;
        double brPower = y + x - ROTATION_SCALE * rx;
        double maxAbs = Math.max(
                Math.max(Math.abs(flPower), Math.abs(blPower)),
                Math.max(Math.abs(frPower), Math.abs(brPower)));
        if (maxAbs > 1.0) {
            flPower /= maxAbs; blPower /= maxAbs;
            frPower /= maxAbs; brPower /= maxAbs;
        }
        frontLeft.setPower(flPower);
        backLeft.setPower(blPower);
        frontRight.setPower(frPower);
        backRight.setPower(brPower);

        // Subsystem updates
        turret.setTargetAngle(baseTarget);
        turret.update(dt);
        flywheel.setTargetRPM(targetFlywheelRPM);
        flywheel.update();
        updateModels();

        // Derived debug values
        double actualRPM        = flywheel.getCurrentRPM();
        double rpmError         = targetFlywheelRPM - actualRPM;
        boolean flywheelAtSpeed = targetFlywheelRPM > 0 && Math.abs(rpmError) < RPM_AT_SPEED_THRESHOLD;
        double turretError      = turret.getTargetAngle() - turret.getCurrentAngle();
        boolean turretSettled   = Math.abs(turretError) < turretAcceptableError;
        double battery          = battSensor.getVoltage();
        double dtMs             = dt * 1000.0;

        // ── POSE ──────────────────────────────────────────────────────────────
        telemetry.addData("=== POSE ===",      "");
        telemetry.addData("X (in)",            f2(robotX));
        telemetry.addData("Y (in)",            f2(robotY));
        telemetry.addData("Heading (deg)",     f2(Math.toDegrees(robotHeading)));
        telemetry.addData("LL last X",         f2(localizer.lastAcceptedX));
        telemetry.addData("LL last Y",         f2(localizer.lastAcceptedY));
        telemetry.addData("dX  pedro-LL",      f2(robotX - localizer.lastAcceptedX));
        telemetry.addData("dY  pedro-LL",      f2(robotY - localizer.lastAcceptedY));

        // ── SHOOTER ───────────────────────────────────────────────────────────
        telemetry.addData("=== SHOOTER ===",   "");
        telemetry.addData("Raw dist (in)",     f2(distanceInches));
        telemetry.addData("Corrected (in)",    f2(correctedDist));
        telemetry.addData("Regime",            shootRegime);
        telemetry.addData("Target RPM",        f0(targetFlywheelRPM));
        telemetry.addData("Actual RPM",        f0(actualRPM));
        telemetry.addData("RPM Error",         f0(rpmError));
        telemetry.addData("At Speed",          flywheelAtSpeed ? "YES" : "NO");
        telemetry.addData("Model RPM",         f0(modelRpm));
        telemetry.addData("Hood cmd",          f3(hood.getPosition()));
        telemetry.addData("Model tilt",        f3(modelTilt));
        telemetry.addData("Stopper cmd",       f3(stopper.getPosition()));

        // ── TURRET ────────────────────────────────────────────────────────────
        telemetry.addData("=== TURRET ===",    "");
        telemetry.addData("Mode",              turretJoystick ? "JOYSTICK" : "AUTO-AIM");
        telemetry.addData("Settled",           turretSettled ? "YES" : "NO");
        telemetry.addData("Target (deg)",      f2(turret.getTargetAngle()));
        telemetry.addData("Current (deg)",     f2(turret.getCurrentAngle()));
        telemetry.addData("Error (deg)",       f2(turretError));
        telemetry.addData("Output",            f4(turret.getOutput()));
        telemetry.addData("Goal dX",           f2(goalDX));
        telemetry.addData("Goal dY",           f2(goalDY));
        telemetry.addData("Field angle (deg)", f2(fieldAngleDeg));
        telemetry.addData("Robot angle (deg)", f2(robotAngleDeg));

        // ── VISION ────────────────────────────────────────────────────────────
        telemetry.addData("=== VISION ===",    "");
        telemetry.addData("Frame status",      localizer.lastFrameStatus);
        telemetry.addData("Staleness (ms)",    f1(localizer.lastStalenessMs));
        telemetry.addData("Tags in view",      localizer.lastTagCount);
        telemetry.addData("Jump (in)",         f2(localizer.lastJumpIn));
        telemetry.addData("Consec rejects",    localizer.consecutiveRejects);
        telemetry.addData("Heading seed",      f2(HeadingConverter.pedroToLimelightDeg(robotHeading)));
        telemetry.addData("FTC X (in)",        f2(localizer.lastFtcX));
        telemetry.addData("FTC Y (in)",        f2(localizer.lastFtcY));

        // ── SYSTEM ────────────────────────────────────────────────────────────
        telemetry.addData("=== SYSTEM ===",    "");
        telemetry.addData("Battery (V)",       f2(battery));
        telemetry.addData("Loop (ms)",         String.format("%.1f %s", dtMs, dtMs > 20 ? "[SLOW]" : ""));
        telemetry.addData("Intake vel (t/s)",  f0(intake.getVelocity()));
        telemetry.addData("Intake state",      intakeState);
        telemetry.addData("Alliance",          allianceRed ? "RED (Y to toggle)" : "BLUE (Y to toggle)");

        telemetry.update();
        telemetryM.update();
    }

    @Override
    public void stop() {
        prism.clearAllAnimations();
        prism.updateAllAnimations();
    }

    private double calculateTurretAngle() {
        double deltaX = FieldPositions.targetGoalX - robotX;
        double deltaY = FieldPositions.targetGoalY - robotY;
        double fieldAngle = Math.atan2(deltaY, deltaX);
        double robotAngle = -(fieldAngle - robotHeading);
        double degrees = Math.toDegrees(robotAngle);
        while (degrees >  180) degrees -= 360;
        while (degrees < -180) degrees += 360;
        return degrees + RobotConstants.TURRET_TARGET_OFFSET;
    }

    private double calculateDistanceToGoal() {
        double dx = FieldPositions.targetGoalX - robotX;
        double dy = FieldPositions.targetGoalY - robotY;
        return Math.sqrt(dx * dx + dy * dy);
    }

    private static void updateModels() {
        RPM_M  = (RobotConstants.RPM_AT_2M - RobotConstants.RPM_AT_1M)
                / (RobotConstants.TRUE_2M_IN - RobotConstants.TRUE_1M_IN);
        RPM_C  = RobotConstants.RPM_AT_1M - RPM_M * RobotConstants.TRUE_1M_IN;
        TILT_M = (RobotConstants.TILT_AT_2M - RobotConstants.TILT_AT_1M)
                / (RobotConstants.TRUE_2M_IN - RobotConstants.TRUE_1M_IN);
        TILT_C = RobotConstants.TILT_AT_1M - TILT_M * RobotConstants.TRUE_1M_IN;
    }

    private static double ticksPerSec(double rpm) { return (145.1 * rpm) / 60.0; }
    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
    private static String f0(double v) { return String.format("%.0f", v); }
    private static String f1(double v) { return String.format("%.1f", v); }
    private static String f2(double v) { return String.format("%.2f", v); }
    private static String f3(double v) { return String.format("%.3f", v); }
    private static String f4(double v) { return String.format("%.4f", v); }

    private void configureLed(PrismAnimations.Solid anim) {
        anim.setBrightness(100);
        anim.setStartIndex(0);
        anim.setStopIndex(36);
    }
}
