package org.firstinspires.ftc.teamcode.lawrence.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.lawrence.constants.HardwareNames;
import org.firstinspires.ftc.teamcode.lawrence.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.lawrence.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.lawrence.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.lawrence.vision.HeadingConverter;

@Configurable
@Autonomous(name = "Red Goal Close Zone 12", group = "Lawrence")
public class RedGoal12 extends OpMode {

    public static double turretFallBackAngle = -42.8;
    public static double flywheelRPM         = 2380;
    public static double shootTime           = 1200;
    public static double waitTime            = 1600;
    public static double classifierTime      = 1400;
    public static double intakeShootRPM      = 1000;
    public static double intakingRPM         = 1100;
    public static double stopperDown         = 0.13;
    public static double stopperUp           = 0;
    public static double hoodUp              = 0.34;
    public static double hoodDown            = 0.03;

    private Follower follower;
    private Flywheel flywheel;
    private Turret turret;
    private DcMotorEx intake;
    private Servo stopper, hood;
    private Limelight3A limelight;
    private GoBildaPrismDriver prism;

    private final PrismAnimations.Solid ledRed   = new PrismAnimations.Solid(Color.RED);
    private final PrismAnimations.Solid ledPink   = new PrismAnimations.Solid(Color.PINK);
    private final PrismAnimations.Solid ledGreen  = new PrismAnimations.Solid(Color.GREEN);

    private Timer pathTimer              = new Timer();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime loopTimer  = new ElapsedTime();
    private double dt = 0;
    private PathState pathState;

    public enum PathState {
        START,
        DRIVE_CLOSESTARTPOS_CLOSESHOOTPOS,
        CLOSESHOOT1,
        DRIVE_CLOSESHOOTPOS_CLOSELOAD,
        CLOSELOAD,
        DRIVE_CLOSELOADENDPOS_CLOSESHOOTPOS,
        CLOSESHOOT2,
        DRIVE_CLOSESHOOTPOS_MIDDLELOAD,
        MIDDLELOAD,
        CLASSIFIERSETUP,
        CLASSIFIEREMPTY,
        DRIVE_CLASSIFIEREMPTYPOS_CLOSESHOOTPOS,
        CLOSESHOOT3,
        DRIVE_CLOSESHOOTPOS_FARLOADSTARTPOS,
        FARLOAD,
        DRIVE_FARLOADENDPOS_CLOSESHOOTPOS,
        CLOSESHOOT4,
        LEAVE
    }

    private final Pose startPose           = new Pose(124,    124,  Math.toRadians(306));
    private final Pose closeZoneShootPose  = new Pose(99,     100,  Math.toRadians(0));
    private final Pose closeLoadStartPose  = new Pose(95,     83.5, Math.toRadians(0));
    private final Pose closeLoadEndPose    = new Pose(126,    83.5, Math.toRadians(0));
    private final Pose middleLoadStartPose = new Pose(96,     58,   Math.toRadians(0));
    private final Pose middleLoadEndPose   = new Pose(133,    58,   Math.toRadians(0));
    private final Pose farLoadStartPose    = new Pose(95,     34,   Math.toRadians(0));
    private final Pose farLoadEndPose      = new Pose(133,    34,   Math.toRadians(0));
    private final Pose classifierSetup     = new Pose(120,    65,   Math.toRadians(10));
    private final Pose classifierEmpty     = new Pose(125.25, 71.5, Math.toRadians(10));
    private final Pose leavePose           = new Pose(110,    88,   Math.toRadians(270));

    private PathChain driveToShoot, driveToCloseLoad, closeLoad,
            driveCloseLoadToShoot, driveToMiddleLoad, middleLoad,
            driveToClassifierSetup, classifierEmptyPath,
            driveClassifierToShoot, driveToFarLoad, farLoadPath,
            driveFarLoadToShoot, driveToLeave;

    private void buildPaths() {
        driveToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, closeZoneShootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), closeZoneShootPose.getHeading()).build();
        driveToCloseLoad = follower.pathBuilder()
                .addPath(new BezierLine(closeZoneShootPose, closeLoadStartPose))
                .setLinearHeadingInterpolation(closeZoneShootPose.getHeading(), closeLoadStartPose.getHeading()).build();
        closeLoad = follower.pathBuilder()
                .addPath(new BezierLine(closeLoadStartPose, closeLoadEndPose))
                .setConstantHeadingInterpolation(closeLoadEndPose.getHeading()).build();
        driveCloseLoadToShoot = follower.pathBuilder()
                .addPath(new BezierLine(closeLoadEndPose, closeZoneShootPose))
                .setConstantHeadingInterpolation(closeZoneShootPose.getHeading()).build();
        driveToMiddleLoad = follower.pathBuilder()
                .addPath(new BezierLine(closeZoneShootPose, middleLoadStartPose))
                .setConstantHeadingInterpolation(middleLoadStartPose.getHeading()).build();
        middleLoad = follower.pathBuilder()
                .addPath(new BezierLine(middleLoadStartPose, middleLoadEndPose))
                .setLinearHeadingInterpolation(middleLoadStartPose.getHeading(), middleLoadEndPose.getHeading()).build();
        driveToClassifierSetup = follower.pathBuilder()
                .addPath(new BezierLine(middleLoadEndPose, classifierSetup))
                .setLinearHeadingInterpolation(middleLoadEndPose.getHeading(), classifierSetup.getHeading()).build();
        classifierEmptyPath = follower.pathBuilder()
                .addPath(new BezierLine(classifierSetup, classifierEmpty))
                .setLinearHeadingInterpolation(classifierSetup.getHeading(), classifierEmpty.getHeading()).build();
        driveClassifierToShoot = follower.pathBuilder()
                .addPath(new BezierLine(classifierEmpty, closeZoneShootPose))
                .setLinearHeadingInterpolation(classifierEmpty.getHeading(), closeZoneShootPose.getHeading()).build();
        driveToFarLoad = follower.pathBuilder()
                .addPath(new BezierLine(closeZoneShootPose, farLoadStartPose))
                .setLinearHeadingInterpolation(closeZoneShootPose.getHeading(), farLoadStartPose.getHeading()).build();
        farLoadPath = follower.pathBuilder()
                .addPath(new BezierLine(farLoadStartPose, farLoadEndPose))
                .setLinearHeadingInterpolation(farLoadStartPose.getHeading(), farLoadEndPose.getHeading()).build();
        driveFarLoadToShoot = follower.pathBuilder()
                .addPath(new BezierLine(farLoadEndPose, closeZoneShootPose))
                .setLinearHeadingInterpolation(farLoadEndPose.getHeading(), closeZoneShootPose.getHeading()).build();
        driveToLeave = follower.pathBuilder()
                .addPath(new BezierLine(closeZoneShootPose, leavePose))
                .setLinearHeadingInterpolation(closeZoneShootPose.getHeading(), leavePose.getHeading()).build();
    }

    private void stateUpdate() {
        switch (pathState) {
            case START:
                follower.followPath(driveToShoot, 0.9, true);
                flywheel.setTargetRPM(flywheelRPM);
                hood.setPosition(hoodUp);
                stateTimer.reset();
                setPathState(PathState.DRIVE_CLOSESTARTPOS_CLOSESHOOTPOS);
                break;
            case DRIVE_CLOSESTARTPOS_CLOSESHOOTPOS:
                turret.setTargetAngle(turretFallBackAngle);
                if (follower.atPose(closeZoneShootPose, 1, 1, 0.05) || stateTimer.milliseconds() > 4000) {
                    if (stateTimer.milliseconds() > waitTime) {
                        stateTimer.reset();
                        setPathState(PathState.CLOSESHOOT1);
                    }
                }
                break;
            case CLOSESHOOT1:
                if (stateTimer.milliseconds() < shootTime) {
                    stopper.setPosition(stopperUp);
                    intake.setVelocity((intakeShootRPM * 145.1) / 60);
                } else {
                    intake.setVelocity(0);
                    stopper.setPosition(Math.min(stopperDown, RobotConstants.STOPPER_DOWN));
                    turret.setTargetAngle(0);
                    flywheel.setTargetRPM(0);
                    follower.followPath(driveToCloseLoad);
                    stateTimer.reset();
                    setPathState(PathState.DRIVE_CLOSESHOOTPOS_CLOSELOAD);
                }
                break;
            case DRIVE_CLOSESHOOTPOS_CLOSELOAD:
                if (!follower.isBusy()) {
                    follower.followPath(closeLoad, 0.5, true);
                    intake.setVelocity((intakingRPM * 145.1) / 60);
                    stateTimer.reset();
                    setPathState(PathState.CLOSELOAD);
                }
                break;
            case CLOSELOAD:
                if (!follower.isBusy()) {
                    intake.setVelocity(0);
                    follower.followPath(driveCloseLoadToShoot, 1, false);
                    stateTimer.reset();
                    setPathState(PathState.DRIVE_CLOSELOADENDPOS_CLOSESHOOTPOS);
                }
                break;
            case DRIVE_CLOSELOADENDPOS_CLOSESHOOTPOS:
                flywheel.setTargetRPM(flywheelRPM);
                turret.setTargetAngle(turretFallBackAngle);
                if (follower.atPose(closeZoneShootPose, 1, 1, 0.05) || stateTimer.milliseconds() > 3000) {
                    stateTimer.reset();
                    setPathState(PathState.CLOSESHOOT2);
                }
                break;
            case CLOSESHOOT2:
                if (stateTimer.milliseconds() < shootTime) {
                    stopper.setPosition(stopperUp);
                    intake.setVelocity((intakeShootRPM * 145.1) / 60);
                } else {
                    intake.setVelocity(0);
                    stopper.setPosition(Math.min(stopperDown, RobotConstants.STOPPER_DOWN));
                    turret.setTargetAngle(0);
                    flywheel.setTargetRPM(0);
                    follower.followPath(driveToMiddleLoad, true);
                    stateTimer.reset();
                    setPathState(PathState.DRIVE_CLOSESHOOTPOS_MIDDLELOAD);
                }
                break;
            case DRIVE_CLOSESHOOTPOS_MIDDLELOAD:
                if (!follower.isBusy()) {
                    follower.followPath(middleLoad, 0.5, true);
                    intake.setVelocity((intakingRPM * 145.1) / 60);
                    stateTimer.reset();
                    setPathState(PathState.MIDDLELOAD);
                }
                break;
            case MIDDLELOAD:
                if (!follower.isBusy()) {
                    intake.setVelocity(0);
                    follower.followPath(driveToClassifierSetup, 0.8, false);
                    stateTimer.reset();
                    setPathState(PathState.CLASSIFIERSETUP);
                }
                break;
            case CLASSIFIERSETUP:
                if (follower.atPose(classifierSetup, 1, 1, 0.5) || stateTimer.milliseconds() > 3000) {
                    follower.followPath(classifierEmptyPath, 0.7, true);
                    stateTimer.reset();
                    setPathState(PathState.CLASSIFIEREMPTY);
                }
                break;
            case CLASSIFIEREMPTY:
                if (!follower.isBusy()) {
                    if (stateTimer.milliseconds() > classifierTime) {
                        follower.followPath(driveClassifierToShoot, true);
                        stateTimer.reset();
                        setPathState(PathState.DRIVE_CLASSIFIEREMPTYPOS_CLOSESHOOTPOS);
                    }
                }
                break;
            case DRIVE_CLASSIFIEREMPTYPOS_CLOSESHOOTPOS:
                flywheel.setTargetRPM(flywheelRPM);
                turret.setTargetAngle(turretFallBackAngle);
                if (follower.atPose(closeZoneShootPose, 1, 1, 0.05) || stateTimer.milliseconds() > 3000) {
                    stateTimer.reset();
                    setPathState(PathState.CLOSESHOOT3);
                }
                break;
            case CLOSESHOOT3:
                if (stateTimer.milliseconds() < shootTime) {
                    stopper.setPosition(stopperUp);
                    intake.setVelocity((intakeShootRPM * 145.1) / 60);
                } else {
                    intake.setVelocity(0);
                    stopper.setPosition(Math.min(stopperDown, RobotConstants.STOPPER_DOWN));
                    turret.setTargetAngle(0);
                    flywheel.setTargetRPM(0);
                    follower.followPath(driveToFarLoad, true);
                    stateTimer.reset();
                    setPathState(PathState.DRIVE_CLOSESHOOTPOS_FARLOADSTARTPOS);
                }
                break;
            case DRIVE_CLOSESHOOTPOS_FARLOADSTARTPOS:
                if (!follower.isBusy()) {
                    follower.followPath(farLoadPath, 0.5, true);
                    intake.setVelocity((intakingRPM * 145.1) / 60);
                    stateTimer.reset();
                    setPathState(PathState.FARLOAD);
                }
                break;
            case FARLOAD:
                if (!follower.isBusy()) {
                    intake.setVelocity(0);
                    follower.followPath(driveFarLoadToShoot, true);
                    stateTimer.reset();
                    setPathState(PathState.DRIVE_FARLOADENDPOS_CLOSESHOOTPOS);
                }
                break;
            case DRIVE_FARLOADENDPOS_CLOSESHOOTPOS:
                flywheel.setTargetRPM(flywheelRPM);
                turret.setTargetAngle(turretFallBackAngle);
                if (follower.atPose(closeZoneShootPose, 1, 1, 0.05) || stateTimer.milliseconds() > 3000) {
                    stateTimer.reset();
                    setPathState(PathState.CLOSESHOOT4);
                }
                break;
            case CLOSESHOOT4:
                if (stateTimer.milliseconds() < shootTime) {
                    stopper.setPosition(stopperUp);
                    intake.setVelocity((intakeShootRPM * 145.1) / 60);
                } else {
                    intake.setVelocity(0);
                    stopper.setPosition(Math.min(stopperDown, RobotConstants.STOPPER_DOWN));
                    turret.setTargetAngle(0);
                    flywheel.setTargetRPM(0);
                    follower.followPath(driveToLeave, true);
                    stateTimer.reset();
                    setPathState(PathState.LEAVE);
                }
                break;
            case LEAVE:
                if (!follower.isBusy()) {
                    prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, ledGreen);
                }
                break;
        }
    }

    private void setPathState(PathState state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        intake  = hardwareMap.get(DcMotorEx.class, HardwareNames.INTAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        stopper = hardwareMap.get(Servo.class, HardwareNames.STOPPER_RIGHT);
        hood    = hardwareMap.get(Servo.class, HardwareNames.HOOD_RIGHT);

        flywheel = new Flywheel(hardwareMap);
        turret   = new Turret(hardwareMap, true);

        limelight = hardwareMap.get(Limelight3A.class, HardwareNames.LIMELIGHT);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(1);

        prism = hardwareMap.get(GoBildaPrismDriver.class, HardwareNames.PRISM);
        configureLed(ledRed);
        configureLed(ledPink);
        configureLed(ledGreen);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, ledRed);

        stopper.setPosition(stopperUp);
        hood.setPosition(hoodDown);

        pathTimer = new Timer();
        buildPaths();
        pathState = PathState.DRIVE_CLOSESTARTPOS_CLOSESHOOTPOS;
    }

    @Override
    public void start() {
        limelight.updateRobotOrientation(HeadingConverter.pedroToLimelightDeg(follower.getPose().getHeading()));
        setPathState(PathState.START);
        stopper.setPosition(Math.min(stopperDown, RobotConstants.STOPPER_DOWN));
        turret.setTargetAngle(turretFallBackAngle);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, ledPink);
        stateTimer.reset();
    }

    @Override
    public void loop() {
        dt = loopTimer.seconds();
        loopTimer.reset();
        follower.update();
        turret.update(dt);
        flywheel.update();
        stateUpdate();
    }

    @Override
    public void stop() {
        prism.clearAllAnimations();
        prism.updateAllAnimations();
    }

    private void configureLed(PrismAnimations.Solid anim) {
        anim.setBrightness(100);
        anim.setStartIndex(0);
        anim.setStopIndex(36);
    }
}
