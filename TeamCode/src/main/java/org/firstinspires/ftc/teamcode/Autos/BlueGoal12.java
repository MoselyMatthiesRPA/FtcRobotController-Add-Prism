package org.firstinspires.ftc.teamcode.Autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DualPidMotor;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "Blue Goal Close zone 12", group = "Competition")
public class BlueGoal12 extends OpMode {
    private Follower follower; // Pedro Pathing follower instance
    private Timer pathTimer;
    ElapsedTime stateTimer = new ElapsedTime();
    GoBildaPrismDriver prism;
    DcMotorEx intake;
    Turret turret;
    Servo rbstop, rhoodtilt;
    Limelight3A limelight;
    DualPidMotor flywheel;
    ElapsedTime loopTimer = new ElapsedTime();
    double dt = 0;
    public static double turretFallBackAngle = 42.8;
    public static double shootTime = 1200;
    public static double waitTime = 1600;
    public static double classifierTime = 1400;
    public static double intakeShootRPM = 1100;
    public static double intakingRPM = 1100;
    public static double stopperDown = 0.16;
    public static double stopperUp = 0;
    public static double hoodUp = 0.34;
    public static double hoodDown = 0.03;
    boolean cameragood;
    public double turretTarget = 2;
    public static double targetoffset = 2;
    public static double flywheelRPM = 2380;
    boolean turretZero = false;


    PrismAnimations.RainbowSnakes rainbow = new PrismAnimations.RainbowSnakes();
    PrismAnimations.Solid solidBlue =new PrismAnimations.Solid(Color.BLUE);
    PrismAnimations.Solid solidPink = new PrismAnimations.Solid(Color.PINK);
    PrismAnimations.Solid solidGreen = new PrismAnimations.Solid(Color.GREEN);
    PathState pathState;
    public enum PathState{
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
        DRIVE_CLASSIFIEREMPTYPOS_FARSHOOTPOS,
        CLOSESHOOT3,
        DRIVE_CLOSESHOOTPOS_FARLOADSTARTPOS,
        FARLOAD,
        DRIVE_FARLOADENDPOS_CLOSESHOOTPOS,
        CLOSESHOOT4,
        LEAVE
    }

    private final Pose startPose = new Pose(20, 124, Math.toRadians(234));
    private final Pose closeZoneShootPose = new Pose (45, 100, Math.toRadians(180));
    private final Pose closeLoadStartPose = new Pose (49, 83.5, Math.toRadians(180));
    private final Pose closeLoadEndPose = new Pose (17, 83.5, Math.toRadians(180));
    private final Pose middleLoadStartPose = new Pose (48, 58, Math.toRadians(180));
    private final Pose middleLoadEndPose = new Pose (11, 58, Math.toRadians(180));
    private final Pose farLoadStartPose = new Pose (48, 35, Math.toRadians(180));
    private final Pose farLoadControlPose = new Pose (44, 55, Math.toRadians(180));
    private final Pose farLoadEndPose = new Pose (11, 35, Math.toRadians(180));
    private final Pose classifierSetup = new Pose (24, 65, Math.toRadians(170));
    private final Pose classifierEmpty = new Pose (18.75, 71.5, Math.toRadians(170));
    private final Pose leavePose = new Pose (34,88,Math.toRadians(270));

    private PathChain CloseStartDriveCloseShoot, CloseShootDriveCloseLoad, CloseLoad, DriveCloseLoadCloseShoot, DriveCloseShootMiddleLoad, MiddleLoad, DriveMiddleLoadClassifierSetup, ClassifierEmpty, DriveClassifierEmptyCloseShoot, DriveCloseShootFarLoad, FarLoad, DriveFarLoadCloseShoot, FarShootLeave;
    public void buildPaths() {
        CloseStartDriveCloseShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, closeZoneShootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), closeZoneShootPose.getHeading())
                .build();
        CloseShootDriveCloseLoad = follower.pathBuilder()
                .addPath(new BezierLine(closeZoneShootPose, closeLoadStartPose))
                .setLinearHeadingInterpolation(closeZoneShootPose.getHeading(), closeLoadStartPose.getHeading())
                .build();
        CloseLoad = follower.pathBuilder()
                .addPath(new BezierLine(closeLoadStartPose, closeLoadEndPose))
                .setConstantHeadingInterpolation(closeLoadEndPose.getHeading())
                .build();
        DriveCloseLoadCloseShoot = follower.pathBuilder()
                .addPath(new BezierLine(closeLoadEndPose, closeZoneShootPose))
                .setConstantHeadingInterpolation(closeZoneShootPose.getHeading())
                .build();
        DriveCloseShootMiddleLoad = follower.pathBuilder()
                .addPath(new BezierLine(closeZoneShootPose, middleLoadStartPose))
                .setConstantHeadingInterpolation(closeLoadStartPose.getHeading())
                .build();
        MiddleLoad = follower.pathBuilder()
                .addPath(new BezierLine(middleLoadStartPose, middleLoadEndPose))
                .setLinearHeadingInterpolation(middleLoadStartPose.getHeading(), middleLoadEndPose.getHeading())
                .build();
        DriveMiddleLoadClassifierSetup = follower.pathBuilder()
                .addPath(new BezierLine(middleLoadEndPose, classifierSetup))
                .setLinearHeadingInterpolation(middleLoadEndPose.getHeading(), classifierSetup.getHeading())
                .build();
        ClassifierEmpty = follower.pathBuilder()
                .addPath(new BezierLine(classifierSetup, classifierEmpty))
                .setLinearHeadingInterpolation(classifierSetup.getHeading(), classifierEmpty.getHeading())
                .build();
        DriveClassifierEmptyCloseShoot = follower.pathBuilder()
                .addPath(new BezierLine(classifierEmpty, closeZoneShootPose))
                .setLinearHeadingInterpolation(classifierEmpty.getHeading(), closeZoneShootPose.getHeading())
                .build();
        DriveCloseShootFarLoad = follower.pathBuilder()
                .addPath(new BezierLine(closeZoneShootPose, farLoadStartPose))
                .setLinearHeadingInterpolation(closeZoneShootPose.getHeading(), farLoadStartPose.getHeading())
                .build();
        FarLoad = follower.pathBuilder()
                .addPath(new BezierLine(farLoadStartPose, farLoadEndPose))
                .setLinearHeadingInterpolation(farLoadStartPose.getHeading(), farLoadEndPose.getHeading())
                .build();
        DriveFarLoadCloseShoot = follower.pathBuilder()
                .addPath(new BezierLine(farLoadEndPose, closeZoneShootPose))
                .setLinearHeadingInterpolation(farLoadEndPose.getHeading(), closeZoneShootPose.getHeading())
                .build();
        FarShootLeave = follower.pathBuilder()
                .addPath(new BezierLine(closeZoneShootPose, leavePose))
                .setLinearHeadingInterpolation(closeZoneShootPose.getHeading(), leavePose.getHeading())
                .build();
    }
    public void statePathUpdate(){
        switch(pathState){
            case START:
                follower.followPath(CloseStartDriveCloseShoot, 0.9, true);
                flywheel.setVelocity(flywheelRPM);
                rhoodtilt.setPosition(hoodUp);
                stateTimer.reset();
                setPathState(PathState.DRIVE_CLOSESTARTPOS_CLOSESHOOTPOS);
                break;

            case DRIVE_CLOSESTARTPOS_CLOSESHOOTPOS:
//                if (follower.atPose(closeZoneShootPose, 1,1, 0.05)) {
                if (!follower.isBusy()){
                    if (stateTimer.milliseconds() > waitTime) {
                        stateTimer.reset();
                        setPathState(PathState.CLOSESHOOT1);
                    }
                }
                break;

            case CLOSESHOOT1:
                if (stateTimer.milliseconds() < shootTime){
                    rbstop.setPosition(stopperUp);
                    intake.setVelocity((intakeShootRPM * 145.1)/60);
                } else if (stateTimer.milliseconds() > shootTime){
                    intake.setVelocity(0);
                    rbstop.setPosition(stopperDown);
//                    turret.setTargetAngle(0);
                    turretZero = true;
                    flywheel.setVelocity(0);
                    follower.followPath(CloseShootDriveCloseLoad);
                    stateTimer.reset();
                    setPathState(PathState.DRIVE_CLOSESHOOTPOS_CLOSELOAD);
                }
                break;
            case DRIVE_CLOSESHOOTPOS_CLOSELOAD:
                if (!follower.isBusy()){
                    follower.followPath(CloseLoad, 0.5, true);
                    intake.setVelocity((intakingRPM * 145.1)/60);
                    stateTimer.reset();
                    setPathState(PathState.CLOSELOAD);
                }
                break;
            case CLOSELOAD:
                if (!follower.isBusy()){
                    intake.setVelocity(0);
                    follower.followPath(DriveCloseLoadCloseShoot, 1, false);
                    stateTimer.reset();
                    setPathState(PathState.DRIVE_CLOSELOADENDPOS_CLOSESHOOTPOS);
                }
                break;
            case DRIVE_CLOSELOADENDPOS_CLOSESHOOTPOS:
                flywheel.setVelocity(flywheelRPM);
                turretZero = false;
//                turret.setTargetAngle(turretFallBackAngle);
                if (follower.atPose(closeZoneShootPose, 1,1, 0.05)) {
                    stateTimer.reset();
                    setPathState(PathState.CLOSESHOOT2);
                }
                break;
            case CLOSESHOOT2:
                if (stateTimer.milliseconds() < shootTime){
                    rbstop.setPosition(stopperUp);
                    intake.setVelocity((intakeShootRPM * 145.1)/60);
                } else if (stateTimer.milliseconds() > shootTime){
                    intake.setVelocity(0);
                    rbstop.setPosition(stopperDown);
//                    turret.setTargetAngle(0);
                    turretZero = true;
                    flywheel.setVelocity(0);
                    follower.followPath(DriveCloseShootMiddleLoad, true);
                    stateTimer.reset();
                    setPathState(PathState.DRIVE_CLOSESHOOTPOS_MIDDLELOAD);
                }
                break;
            case DRIVE_CLOSESHOOTPOS_MIDDLELOAD:
                if (!follower.isBusy()){
                    follower.followPath(MiddleLoad, 0.5, true);
                    intake.setVelocity((intakingRPM * 145.1)/60);
                    stateTimer.reset();
                    setPathState(PathState.MIDDLELOAD);
                }
                break;
            case MIDDLELOAD:
                if (!follower.isBusy()){
                    intake.setVelocity(0);
                    follower.followPath(DriveMiddleLoadClassifierSetup, 0.8, false);
                    stateTimer.reset();
                    setPathState(PathState.CLASSIFIERSETUP);
                }
                break;
            case CLASSIFIERSETUP:
                if (follower.atPose(classifierSetup, 1, 1, 0.5)){
                    follower.followPath(ClassifierEmpty, 0.7, true);
                    stateTimer.reset();
                    setPathState(PathState.CLASSIFIEREMPTY);
                }
                break;
            case CLASSIFIEREMPTY:
                if (!follower.isBusy()) {
                    if (stateTimer.milliseconds() > classifierTime) {
                        follower.followPath(DriveClassifierEmptyCloseShoot, true);
                        stateTimer.reset();
                        setPathState(PathState.DRIVE_CLASSIFIEREMPTYPOS_FARSHOOTPOS);
                    }
                }
                break;
            case DRIVE_CLASSIFIEREMPTYPOS_FARSHOOTPOS:
                flywheel.setVelocity(flywheelRPM);
//                turret.setTargetAngle(turretFallBackAngle);
                turretZero = false;
                if (follower.atPose(closeZoneShootPose, 1,1, 0.05)) {
                    stateTimer.reset();
                    setPathState(PathState.CLOSESHOOT3);
                }
                break;
            case CLOSESHOOT3:
                if (stateTimer.milliseconds() < shootTime){
                    rbstop.setPosition(stopperUp);
                    intake.setVelocity((intakeShootRPM * 145.1)/60);
                } else if (stateTimer.milliseconds() > shootTime){
                    intake.setVelocity(0);
                    rbstop.setPosition(stopperDown);
//                    turret.setTargetAngle(0);
                    turretZero = true;
                    flywheel.setVelocity(0);
                    follower.followPath(DriveCloseShootFarLoad, true);
                    stateTimer.reset();
                    setPathState(PathState.DRIVE_CLOSESHOOTPOS_FARLOADSTARTPOS);
                }
                break;
            case DRIVE_CLOSESHOOTPOS_FARLOADSTARTPOS:
                if (!follower.isBusy()){
                    follower.followPath(FarLoad, 0.5,true);
                    intake.setVelocity((intakingRPM * 145.1)/60);
                    stateTimer.reset();
                    setPathState(PathState.FARLOAD);
                }
                break;
            case FARLOAD:
                if (!follower.isBusy()){
                    intake.setVelocity(0);
                    follower.followPath(DriveFarLoadCloseShoot, true);
                    stateTimer.reset();
                    setPathState(PathState.DRIVE_FARLOADENDPOS_CLOSESHOOTPOS);
                }
                break;
            case DRIVE_FARLOADENDPOS_CLOSESHOOTPOS:
                flywheel.setVelocity(flywheelRPM);
//                turret.setTargetAngle(turretFallBackAngle);
                turretZero = false;
                if (follower.atPose(closeZoneShootPose, 1,1, 0.05)) {
                    stateTimer.reset();
                    setPathState(PathState.CLOSESHOOT4);
                }
                break;
            case CLOSESHOOT4:
                if (stateTimer.milliseconds() < shootTime){
                    rbstop.setPosition(stopperUp);
                    intake.setVelocity((intakeShootRPM * 145.1)/60);
                } else if (stateTimer.milliseconds() > shootTime){
                    intake.setVelocity(0);
                    rbstop.setPosition(stopperDown);
//                    turret.setTargetAngle(0);
                    turretZero = true;
                    flywheel.setVelocity(0);
                    follower.followPath(FarShootLeave, true);
                    stateTimer.reset();
                    setPathState(PathState.LEAVE);
                }
                break;
            case LEAVE:
                if (!follower.isBusy()){
                }
            default:
                break;
        }
    }

    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_CLOSESTARTPOS_CLOSESHOOTPOS;
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        turret = new Turret(hardwareMap, true);
        rbstop = hardwareMap.get(Servo.class, "rbstop");
        rhoodtilt = hardwareMap.get(Servo.class, "rhoodtilt");
        flywheel = new DualPidMotor (hardwareMap, "topflywheel", "bottomflywheel");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        prism = hardwareMap.get(GoBildaPrismDriver.class,"prism");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(1); // Switch to pipeline number 1

        rbstop.setPosition(stopperUp);
        rhoodtilt.setPosition(hoodDown);

        solidBlue.setBrightness(100);
        solidBlue.setStartIndex(0);
        solidBlue.setStopIndex(36);

        solidPink.setBrightness(100);
        solidPink.setStartIndex(0);
        solidPink.setStopIndex(36);

        rainbow.setNumberOfSnakes(3);
        rainbow.setSnakeLength(3);
        rainbow.setSpacingBetween(2);
        rainbow.setSpeed(0.6f);

        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solidBlue);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    public void start() {
        setPathState(PathState.START);
        rbstop.setPosition(stopperDown);
        turret.setTargetAngle(turretFallBackAngle);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solidPink);
        stateTimer.reset();
    }

    @Override
    public void loop() {
        dt = loopTimer.seconds();
        turret.update(dt);
        flywheel.Update();
        loopTimer.reset();
        follower.update();
        statePathUpdate();

        double turretAngle = turret.getCurrentAngle();
        double targetTurretangle = turret.getTargetAngle();
        LLResult result = limelight.getLatestResult();
//        if (turretZero) {
//            if (result != null && result.isValid()) {
//                double lastGoodtTx = result.getTx(); // angle from tag camera-relative(from limelight)
//                turretTarget = turretAngle + targetTurretangle - lastGoodtTx;
//            } else {
//                turretTarget = turretFallBackAngle;
//            }
//        } else {
//            turretTarget = 0;
//        }
        turretTarget = turretFallBackAngle;
    }


    @Override
    public void stop() {
        prism.clearAllAnimations();
        prism.updateAllAnimations();
    }

}