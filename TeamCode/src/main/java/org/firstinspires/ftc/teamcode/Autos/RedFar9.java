package org.firstinspires.ftc.teamcode.Autos;

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

import org.firstinspires.ftc.teamcode.DualPidMotor;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "Red Far zone 9", group = "Competition")
public class RedFar9 extends OpMode {
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
    public static double turretFallBackAngle = -65.5;
    public static double shootTime = 2200;
    public static double waitTime = 1700;
    public static double classifierTime = 2000;
    public static double intakeShootRPM = 800;
    public static double intakingRPM = 1100;
    public static double stopperDown = 0.16;
    public static double stopperUp = 0;
    public static double hoodUp = 0.75;
    public static double hoodDown = 0.03;
    boolean cameragood;
    public double turretTarget = -2;
    public static double targetoffset = 2;


    PrismAnimations.RainbowSnakes rainbow = new PrismAnimations.RainbowSnakes();
    PrismAnimations.Solid solidRed =new PrismAnimations.Solid(Color.RED);
    PrismAnimations.Solid solidPink = new PrismAnimations.Solid(Color.PINK);
    PrismAnimations.Solid solidGreen = new PrismAnimations.Solid(Color.GREEN);
    PathState pathState;
    public enum PathState{
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

    private final Pose startPose = new Pose(95, 7.2, Math.toRadians(0));
    private final Pose farZoneShootPose = new Pose (86, 16, Math.toRadians(0));
    private final Pose middleLoadStartPose = new Pose (96, 58, Math.toRadians(0));
    private final Pose middleLoadEndPose = new Pose (134, 58, Math.toRadians(0));
    private final Pose farLoadStartPose = new Pose (95, 34, Math.toRadians(0));
    private final Pose middleLoadControlPose = new Pose (105, 60, Math.toRadians(0));
    private final Pose farLoadEndPose = new Pose (134, 34, Math.toRadians(0));
    private final Pose classifierSetup = new Pose (120, 65, Math.toRadians(10));
    private final Pose classifierEmpty = new Pose (128.25, 71.5, Math.toRadians(10));
    private final Pose farLeavePose = new Pose (120, 10, Math.toRadians(0));

    private PathChain FarStartDriveFarShoot, FarShootDriveMiddleLoad, MiddleLoad, DriveMiddleLoadClassifierSetup, ClassifierEmpty, DriveClassifierEmptyFarShoot, DriveFarShootFarLoad, FarLoad, DriveFarLoadFarShoot, FarShootLeave;
    public void buildPaths() {
        FarStartDriveFarShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, farZoneShootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), farZoneShootPose.getHeading())
                .build();
        FarShootDriveMiddleLoad = follower.pathBuilder()
                .addPath(new BezierLine(farZoneShootPose, middleLoadStartPose))
                .setLinearHeadingInterpolation(farZoneShootPose.getHeading(), middleLoadStartPose.getHeading())
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
        DriveClassifierEmptyFarShoot = follower.pathBuilder()
                .addPath(new BezierCurve(classifierEmpty, middleLoadControlPose, farZoneShootPose))
                .setLinearHeadingInterpolation(classifierEmpty.getHeading(), farZoneShootPose.getHeading())
                .build();
        DriveFarShootFarLoad = follower.pathBuilder()
                .addPath(new BezierLine(farZoneShootPose, farLoadStartPose))
                .setLinearHeadingInterpolation(farZoneShootPose.getHeading(), farLoadStartPose.getHeading())
                .build();
        FarLoad = follower.pathBuilder()
                .addPath(new BezierLine(farLoadStartPose, farLoadEndPose))
                .setLinearHeadingInterpolation(farLoadStartPose.getHeading(), farLoadEndPose.getHeading())
                .build();
        DriveFarLoadFarShoot = follower.pathBuilder()
                .addPath(new BezierLine(farLoadEndPose, farZoneShootPose))
                .setLinearHeadingInterpolation(farLoadEndPose.getHeading(), farZoneShootPose.getHeading())
                .build();
        FarShootLeave = follower.pathBuilder()
                .addPath(new BezierLine(farZoneShootPose, farLeavePose))
                .setLinearHeadingInterpolation(farZoneShootPose.getHeading(), farLeavePose.getHeading())
                .build();
    }
    public void statePathUpdate(){
        switch(pathState){
            case START:
                follower.followPath(FarStartDriveFarShoot, 0.6, true);
                flywheel.setVelocity(3175);
                rhoodtilt.setPosition(hoodUp);
                stateTimer.reset();
                setPathState(PathState.DRIVE_FARSTARTPOS_FARSHOOTPOS);
                break;

            case DRIVE_FARSTARTPOS_FARSHOOTPOS:
                turret.setTargetAngle(turretTarget);
                if (follower.atPose(farZoneShootPose, 1,1, 0.05)) {
                    if (stateTimer.milliseconds() > waitTime) {
                        stateTimer.reset();
                        setPathState(PathState.FARSHOOT1);
                    }
                }
                break;

            case FARSHOOT1:
                if (stateTimer.milliseconds() < shootTime){
                    rbstop.setPosition(stopperUp);
                    intake.setVelocity((intakeShootRPM * 145.1)/60);
                } else if (stateTimer.milliseconds() > shootTime){
                    intake.setVelocity(0);
                    rbstop.setPosition(stopperDown);
                    turret.setTargetAngle(0);
                    flywheel.setVelocity(0);
                    follower.followPath(FarShootDriveMiddleLoad);
                    stateTimer.reset();
                setPathState(PathState.DRIVE_FARSHOOTPOS_MIDDLELOAD);
                }

            break;
            case DRIVE_FARSHOOTPOS_MIDDLELOAD:
                if (!follower.isBusy()){
                    follower.followPath(MiddleLoad, 0.5, false);
                    intake.setVelocity((intakingRPM * 145.1)/60);
                    stateTimer.reset();
                    setPathState(PathState.MIDDLELOAD);
                }
                break;
            case MIDDLELOAD:
                if (!follower.isBusy()){
                    intake.setVelocity(0);
                    follower.followPath(DriveMiddleLoadClassifierSetup, 0.7, false);
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
                        follower.followPath(DriveClassifierEmptyFarShoot, true);
                        stateTimer.reset();
                        setPathState(PathState.DRIVE_CLASSIFIEREMPTYPOS_FARSHOOTPOS);
                    }
                }
                break;
            case DRIVE_CLASSIFIEREMPTYPOS_FARSHOOTPOS:
                flywheel.setVelocity(3175);
                turret.setTargetAngle(turretFallBackAngle);
                if (follower.atPose(farZoneShootPose, 1,1, 0.05)) {
                        stateTimer.reset();
                        setPathState(PathState.FARSHOOT2);
                    }
                break;
            case FARSHOOT2:
                if (stateTimer.milliseconds() < shootTime){
                    rbstop.setPosition(stopperUp);
                    intake.setVelocity((intakeShootRPM * 145.1)/60);
                } else if (stateTimer.milliseconds() > shootTime){
                    intake.setVelocity(0);
                    rbstop.setPosition(stopperDown);
                    turret.setTargetAngle(0);
                    flywheel.setVelocity(0);
                    follower.followPath(DriveFarShootFarLoad, true);
                    stateTimer.reset();
                    setPathState(PathState.DRIVE_FARSHOOTPOS_FARLOADSTARTPOS);
                }
                break;
            case DRIVE_FARSHOOTPOS_FARLOADSTARTPOS:
                if (!follower.isBusy()){
                    follower.followPath(FarLoad, 0.5, true);
                    intake.setVelocity((intakingRPM * 145.1)/60);
                    stateTimer.reset();
                    setPathState(PathState.FARLOAD);
                }
                break;
            case FARLOAD:
                if (!follower.isBusy()){
                    intake.setVelocity(0);
                    follower.followPath(DriveFarLoadFarShoot, true);
                    stateTimer.reset();
                    setPathState(PathState.DRIVE_FARLOADENDPOS_FARSHOOTPOS);
                }
                break;
            case DRIVE_FARLOADENDPOS_FARSHOOTPOS:
                flywheel.setVelocity(3175);
                turret.setTargetAngle(turretFallBackAngle);
                if (follower.atPose(farZoneShootPose, 1,1, 0.05)) {
                    stateTimer.reset();
                    setPathState(PathState.FARSHOOT3);
                }
                break;
            case FARSHOOT3:
                if (stateTimer.milliseconds() < shootTime){
                    rbstop.setPosition(stopperUp);
                    intake.setVelocity((intakeShootRPM * 145.1)/60);
                } else if (stateTimer.milliseconds() > shootTime){
                    intake.setVelocity(0);
                    rbstop.setPosition(stopperDown);
                    turret.setTargetAngle(0);
                    flywheel.setVelocity(0);
                    follower.followPath(FarShootLeave, true);
                    stateTimer.reset();
                    setPathState(PathState.LEAVE);
                }
                break;
            case LEAVE:
                if (!follower.isBusy()){
                    prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solidGreen);
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
        pathState = PathState.DRIVE_FARSTARTPOS_FARSHOOTPOS;
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

        solidRed.setBrightness(100);
        solidRed.setStartIndex(0);
        solidRed.setStopIndex(36);

        solidPink.setBrightness(100);
        solidPink.setStartIndex(0);
        solidPink.setStopIndex(36);

        rainbow.setNumberOfSnakes(3);
        rainbow.setSnakeLength(3);
        rainbow.setSpacingBetween(2);
        rainbow.setSpeed(0.6f);

        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solidRed);
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

//        double turretAngle = turret.getCurrentAngle();
//        double targetTurretangle = turret.getTargetAngle();
//        LLResult result = limelight.getLatestResult();
//        if (result != null && result.isValid()) {
//            double lastGoodtTx = result.getTx(); // angle from tag camera-relative(from limelight)
//            turretTarget = turretAngle + targetTurretangle + targetoffset - lastGoodtTx; // Calculates the angle the turret would be at if it were perfectly aligned
//
//        } else {
//            turretTarget = turretFallBackAngle;
//        }
        turretTarget = turretFallBackAngle;
        }


    @Override
    public void stop() {
        prism.clearAllAnimations();
        prism.updateAllAnimations();
    }

}