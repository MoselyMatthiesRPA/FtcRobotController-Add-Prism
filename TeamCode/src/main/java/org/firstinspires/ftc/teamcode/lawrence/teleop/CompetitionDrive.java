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

import java.util.List;

@Configurable
@TeleOp(name = "Competition Drive", group = "Lawrence")
public class CompetitionDrive extends OpMode {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private DcMotorEx intake;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private Flywheel flywheel;
    private Turret turret;
    private Servo stopper, hood;
    private Limelight3A limelight;
    private GoBildaPrismDriver prism;

    // ── Subsystems ────────────────────────────────────────────────────────────
    private Follower follower;
    private LimelightLocalizer localizer;
    private TelemetryManager telemetryM;

    // ── LED animations ────────────────────────────────────────────────────────
    private final PrismAnimations.Solid ledGreen = new PrismAnimations.Solid(Color.GREEN);
    private final PrismAnimations.Solid ledPink  = new PrismAnimations.Solid(Color.PINK);
    private final PrismAnimations.Solid ledRed   = new PrismAnimations.Solid(Color.RED);

    // ── Robot pose (updated each loop from Pedro) ─────────────────────────────
    private double robotX       = 0;
    private double robotY       = 0;
    private double robotHeading = Math.PI / 2.0; // radians

    // ── Shooter state ─────────────────────────────────────────────────────────
    private double targetFlywheelRPM = 0;
    private double targetHoodTilt    = RobotConstants.MIN_TILT;
    private double distanceInches      = 0;
    private double intakingRPM         = 0;

    // ── Shooter linear models (recomputed each loop) ──────────────────────────
    private static double RPM_M, RPM_C, TILT_M, TILT_C;

    // ── Turret state ──────────────────────────────────────────────────────────
    private double baseTarget = 0;
    public static double turretAcceptableError = 0.5;

    // ── Configurable turret control ───────────────────────────────────────────
    // Joystick deadzone
    public static double JOYSTICK_DEADZONE = 0.5;
    // Rotation power scale applied to right stick X during drive
    public static double ROTATION_SCALE = 0.8;

    // ── Loop timing ───────────────────────────────────────────────────────────
    private final ElapsedTime loopTimer = new ElapsedTime();

    // ── Starting pose — set before init() if needed by another OpMode ─────────
    public static Pose startingPose = null;

    // ── Misc state ────────────────────────────────────────────────────────────
    private boolean override = false;

    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void init() {
        // Drive motors
        frontLeft  = hardwareMap.get(DcMotor.class, HardwareNames.FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotor.class, HardwareNames.FRONT_RIGHT);
        backLeft   = hardwareMap.get(DcMotor.class, HardwareNames.BACK_LEFT);
        backRight  = hardwareMap.get(DcMotor.class, HardwareNames.BACK_RIGHT);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Intake
        intake = hardwareMap.get(DcMotorEx.class, HardwareNames.INTAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        // Servos
        stopper = hardwareMap.get(Servo.class, HardwareNames.STOPPER_RIGHT);
        hood    = hardwareMap.get(Servo.class, HardwareNames.HOOD_RIGHT);
        hood.setDirection(Servo.Direction.REVERSE);

        // Subsystems
        flywheel = new Flywheel(hardwareMap);
        turret   = new Turret(hardwareMap, true);

        // Pedro Pathing
        follower = Constants.createFollower(hardwareMap);
        Pose start = (startingPose != null) ? startingPose : new Pose(72, 24, Math.toRadians(90));
        follower.setStartingPose(start);
        follower.update();

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, HardwareNames.LIMELIGHT);
        limelight.pipelineSwitch(0);
        limelight.start();
        // Seed yaw before the first MT2 solve so we don't get a mirrored position
        limelight.updateRobotOrientation(
                HeadingConverter.pedroToLimelightDeg(follower.getPose().getHeading()));

        localizer = new LimelightLocalizer(limelight, follower);

        // LED
        prism = hardwareMap.get(GoBildaPrismDriver.class, HardwareNames.PRISM);
        configureLed(ledGreen);
        configureLed(ledPink);
        configureLed(ledRed);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, ledGreen);

        // Servo init positions
        stopper.setPosition(RobotConstants.STOPPER_UP);
        hood.setPosition(RobotConstants.MIN_TILT);

        // Bulk caching — reduces I²C reads per loop
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

        // ── Odometry ──────────────────────────────────────────────────────────
        follower.update();
        Pose pose = follower.getPose();
        robotX       = pose.getX();
        robotY       = pose.getY();
        robotHeading = pose.getHeading();

        // ── Vision pose fusion ────────────────────────────────────────────────
        localizer.update(robotHeading);

        // ── Distance & shooter model ──────────────────────────────────────────
        distanceInches = calculateDistanceToGoal();
        double correctedDist = RobotConstants.DIST_A * distanceInches + RobotConstants.DIST_B;

        if (correctedDist < RobotConstants.FAR_THRESHOLD_IN) {
            targetFlywheelRPM = clamp(RPM_M * correctedDist + RPM_C,
                    RobotConstants.MIN_RPM, RobotConstants.MAX_RPM);
            targetHoodTilt = clamp(TILT_M * correctedDist + TILT_C,
                    RobotConstants.MIN_TILT, RobotConstants.MAX_TILT);
            intakingRPM = RobotConstants.CLOSE_RPM;
        } else {
            targetFlywheelRPM = RobotConstants.RPM_AT_FAR;
            targetHoodTilt    = RobotConstants.TILT_AT_FAR;
            intakingRPM       = RobotConstants.FAR_RPM;
        }
        // ── Turret targeting ──────────────────────────────────────────────────
        double cx = gamepad2.right_stick_x;
        double cy = -gamepad2.right_stick_y;
        boolean turretJoystick = Math.hypot(cx, cy) > JOYSTICK_DEADZONE;

        if (turretJoystick) {
            baseTarget = Math.toDegrees(Math.atan2(cx, cy));
        } else {
            baseTarget = calculateTurretAngle();
        }

        // ── Intake & shooter servo control ────────────────────────────────────
        if (gamepad2.right_trigger > 0.1) {
            stopper.setPosition(RobotConstants.STOPPER_DOWN);
            intake.setVelocity(ticksPerSec(intakingRPM));
        } else if (gamepad2.left_trigger > 0.1) {
            hood.setPosition(RobotConstants.MIN_TILT);
            intake.setVelocity(ticksPerSec(RobotConstants.INTAKING_RPM));
        } else {
            intake.setVelocity(0);
            stopper.setPosition(RobotConstants.STOPPER_UP);
            hood.setPosition(targetHoodTilt);
        }

        // ── Mecanum drive ─────────────────────────────────────────────────────
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
            flPower /= maxAbs;
            blPower /= maxAbs;
            frPower /= maxAbs;
            brPower /= maxAbs;
        }

        frontLeft.setPower(flPower);
        backLeft.setPower(blPower);
        frontRight.setPower(frPower);
        backRight.setPower(brPower);

        // ── Subsystem updates ─────────────────────────────────────────────────
        turret.setTargetAngle(baseTarget);
        turret.update(dt);
        flywheel.setTargetRPM(targetFlywheelRPM);
        flywheel.update();
        updateModels();

        // ── Telemetry ─────────────────────────────────────────────────────────
        telemetry.addData("Turret angle",      turret.getCurrentAngle());
        telemetry.addData("Turret target",     turret.getTargetAngle());
        telemetry.addData("Turret output",     turret.getOutput());
        telemetry.addData("Flywheel RPM",      flywheel.getCurrentRPM());
        telemetry.addData("Target RPM",        targetFlywheelRPM);
        telemetry.addData("Hood tilt",         targetHoodTilt);
        telemetry.addData("Distance (in)",     correctedDist);
        telemetry.addData("Loop time (ms)",    dt * 1000);
        telemetry.addData("Robot X",           robotX);
        telemetry.addData("Robot Y",           robotY);
        telemetry.addData("Heading (deg)",     Math.toDegrees(robotHeading));
        telemetry.update();
        telemetryM.update();
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

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

    // Convert intake RPM to encoder ticks/sec (145.1 ticks/rev GoBILDA)
    private static double ticksPerSec(double rpm) {
        return (145.1 * rpm) / 60.0;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private void configureLed(PrismAnimations.Solid anim) {
        anim.setBrightness(100);
        anim.setStartIndex(0);
        anim.setStopIndex(36);
    }
}
