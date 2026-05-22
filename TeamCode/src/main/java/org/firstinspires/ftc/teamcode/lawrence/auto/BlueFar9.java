package org.firstinspires.ftc.teamcode.lawrence.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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
@Autonomous(name = "Blue Far Zone 9", group = "Lawrence")
public class BlueFar9 extends OpMode {

    public static double turretFallBackAngle = 65.5;
    public static double shootTime           = 2200;
    public static double waitTime            = 1700;
    public static double classifierTime      = 2000;
    public static double intakeShootRPM      = 800;
    public static double intakingRPM         = 1100;
    public static double stopperDown         = 0.13;
    public static double stopperUp           = 0;
    public static double hoodUp              = 0.75;
    public static double hoodDown            = 0.03;

    private Follower follower;
    private Flywheel flywheel;
    private Turret turret;
    private DcMotorEx intake;
    private Servo stopper, hood;
    private Limelight3A limelight;
    private GoBildaPrismDriver prism;

    private final PrismAnimations.Solid ledBlue  = new PrismAnimations.Solid(Color.BLUE);
    private final PrismAnimations.Solid ledPink   = new PrismAnimations.Solid(Color.PINK);
    private final PrismAnimations.Solid ledGreen  = new PrismAnimations.Solid(Color.GREEN);

    private Timer pathTimer              = new Timer();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime loopTimer  = new ElapsedTime();
    private double dt = 0;
    private PathState pathState;

    public enum PathState {
        START,
        DRIVE_FARSTARTPOS_FARSHOOTPOS,
        FARSHOOT1,
        DRIVE_FARSHOOTPOS_MIDDLELOAD,
        MIDDLELOAD,
        CLASSIFIERSETUP,
        CLASSIFIEREMPTY,
        DRIVE_CLASSIFIEREMPTYPOS_FARSHOOTPOS,
        FARSHOOT2,
        DRIVE_FARSHOOTPOS_FARLOADSTARTPOS,
        FARLOAD,
        DRIVE_FARLOADENDPOS_FARSHOOTPOS,
        FARSHOOT3,
        LEAVE
    }

    private final Pose startPose             = new Pose(49,    7.2,  Math.toRadians(180));
    private final Pose farZoneShootPose      = new Pose(58,    16,   Math.toRadians(180));
    private final Pose middleLoadStartPose   = new Pose(48,    58,   Math.toRadians(180));
    private final Pose middleLoadEndPose     = new Pose(10,    58,   Math.toRadians(180));
    private final Pose middleLoadControlPose = new Pose(48,    58,   Math.toRadians(180));
    private final Pose farLoadStartPose      = new Pose(58,    35,   Math.toRadians(180));
    private final Pose farLoadEndPose        = new Pose(10,    35,   Math.toRadians(180));
    private final Pose classifierSetup       = new Pose(24,    65,   Math.toRadians(170));
    private final Pose classifierEmpty       = new Pose(17.75, 71.5, Math.toRadians(170));
    private final Pose farLeavePose          = new Pose(24,    10,   Math.toRadians(180));

    private PathChain driveToShoot, driveToMiddleLoad, middleLoad,
            driveToClassifierSetup, classifierEmptyPath,
            driveClassifierToShoot, driveToFarLoad, farLoadPath,
            driveFarLoadToShoot, driveToLeave;

    private void buildPaths() {
        driveToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, farZoneShootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), farZoneShootPose.getHeading()).build();
        driveToMiddleLoad = follower.pathBuilder()
                .addPath(new BezierLine(farZoneShootPose, middleLoadStartPose))
                .setLinearHeadingInterpolation(farZoneShootPose.getHeading(), middleLoadStartPose.getHeading()).build();
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
                .addPath(new BezierCurve(classifierEmpty, middleLoadControlPose, farZoneShootPose))
                .setLinearHeadingInterpolation(classifierEmpty.getHeading(), farZoneShootPose.getHeading()).build();
        driveToFarLoad = follower.pathBuilder()
                .addPath(new BezierLine(farZoneShootPose, farLoadStartPose))
                .setLinearHeadingInterpolation(farZoneShootPose.getHeading(), farLoadStartPose.getHeading()).build();
        farLoadPath = follower.pathBuilder()
                .addPath(new BezierLine(farLoadStartPose, farLoadEndPose))
                .setLinearHeadingInterpolation(farLoadStartPose.getHeading(), farLoadEndPose.getHeading()).build();
        driveFarLoadToShoot = follower.pathBuilder()
                .addPath(new BezierLine(farLoadEndPose, farZoneShootPose))
                .setLinearHeadingInterpolation(farLoadEndPose.getHeading(), farZoneShootPose.getHeading()).build();
        driveToLeave = follower.pathBuilder()
                .addPath(new BezierLine(farZoneShootPose, farLeavePose))
                .setLinearHeadingInterpolation(farZoneShootPose.getHeading(), farLeavePose.getHeading()).build();
    }

    private void stateUpdate() {
        switch (pathState) {
            case START:
                follower.followPath(driveToShoot, 0.7, true);
                flywheel.setTargetRPM(3075);
                hood.setPosition(hoodUp);
                stateTimer.reset();
                setPathState(PathState.DRIVE_FARSTARTPOS_FARSHOOTPOS);
                break;
            case DRIVE_FARSTARTPOS_FARSHOOTPOS:
                turret.setTargetAngle(turretFallBackAngle);
                if (!follower.isBusy()) {
                    if (stateTimer.milliseconds() > waitTime) {
                        stateTimer.reset();
                        setPathState(PathState.FARSHOOT1);
                    }
                }
                break;
            case FARSHOOT1:
                if (stateTimer.milliseconds() < shootTime) {
                    stopper.setPosition(stopperUp);
                    intake.setVelocity((intakeShootRPM * 145.1) / 60);
                } else {
                    intake.setVelocity(0);
                    stopper.setPosition(Math.min(stopperDown, RobotConstants.STOPPER_DOWN));
                    hood.setPosition(hoodDown);
                    turret.setTargetAngle(0);
                    flywheel.setTargetRPM(0);
                    follower.followPath(driveToMiddleLoad);
                    stateTimer.reset();
                    setPathState(PathState.DRIVE_FARSHOOTPOS_MIDDLELOAD);
                }
                break;
            case DRIVE_FARSHOOTPOS_MIDDLELOAD:
                if (!follower.isBusy()) {
                    follower.followPath(middleLoad, 0.5, false);
                    intake.setVelocity((intakingRPM * 145.1) / 60);
                    stateTimer.reset();
                    setPathState(PathState.MIDDLELOAD);
                }
                break;
            case MIDDLELOAD:
                if (!follower.isBusy()) {
                    intake.setVelocity(0);
                    follower.followPath(driveToClassifierSetup, 0.7, false);
                    stateTimer.reset();
                    setPathState(PathState.CLASSIFIERSETUP);
                }
                break;
            case CLASSIFIERSETUP:
                if (!follower.isBusy()) {
                    follower.followPath(classifierEmptyPath, 0.7, true);
                    stateTimer.reset();
                    setPathState(PathState.CLASSIFIEREMPTY);
                }
                break;
            case CLASSIFIEREMPTY:
                if (!follower.isBusy()) {
                    if (stateTimer.milliseconds() > classifierTime) {
                        follower.followPath(driveClassifierToShoot, 1, true);
                        stateTimer.reset();
                        setPathState(PathState.DRIVE_CLASSIFIEREMPTYPOS_FARSHOOTPOS);
                    }
                }
                break;
            case DRIVE_CLASSIFIEREMPTYPOS_FARSHOOTPOS:
                flywheel.setTargetRPM(3075);
                hood.setPosition(hoodUp);
                turret.setTargetAngle(turretFallBackAngle);
                if (!follower.isBusy()) {
                    stateTimer.reset();
                    setPathState(PathState.FARSHOOT2);
                }
                break;
            case FARSHOOT2:
                if (stateTimer.milliseconds() < shootTime) {
                    stopper.setPosition(stopperUp);
                    intake.setVelocity((intakeShootRPM * 145.1) / 60);
                } else {
                    intake.setVelocity(0);
                    stopper.setPosition(Math.min(stopperDown, RobotConstants.STOPPER_DOWN));
                    hood.setPosition(hoodDown);
                    turret.setTargetAngle(0);
                    flywheel.setTargetRPM(0);
                    follower.followPath(driveToFarLoad, true);
                    stateTimer.reset();
                    setPathState(PathState.DRIVE_FARSHOOTPOS_FARLOADSTARTPOS);
                }
                break;
            case DRIVE_FARSHOOTPOS_FARLOADSTARTPOS:
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
                    setPathState(PathState.DRIVE_FARLOADENDPOS_FARSHOOTPOS);
                }
                break;
            case DRIVE_FARLOADENDPOS_FARSHOOTPOS:
                flywheel.setTargetRPM(3075);
                hood.setPosition(hoodUp);
                turret.setTargetAngle(turretFallBackAngle);
                if (!follower.isBusy()) {
                    stateTimer.reset();
                    setPathState(PathState.FARSHOOT3);
                }
                break;
            case FARSHOOT3:
                if (stateTimer.milliseconds() < shootTime) {
                    stopper.setPosition(stopperUp);
                    intake.setVelocity((intakeShootRPM * 145.1) / 60);
                } else {
                    intake.setVelocity(0);
                    hood.setPosition(hoodDown);
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
        configureLed(ledBlue);
        configureLed(ledPink);
        configureLed(ledGreen);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, ledBlue);

        stopper.setPosition(stopperUp);
        hood.setPosition(hoodDown);

        pathTimer = new Timer();
        buildPaths();
        pathState = PathState.DRIVE_FARSTARTPOS_FARSHOOTPOS;
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
