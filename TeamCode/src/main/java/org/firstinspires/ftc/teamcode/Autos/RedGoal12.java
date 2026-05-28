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
import org.firstinspires.ftc.teamcode.NewBlueGoal;
import org.firstinspires.ftc.teamcode.NewRedGoal;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "Red Goal Close zone 12 artifact", group = "Competition")
public class RedGoal12 extends OpMode {
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
    public static double turretFallBackAngle = -42.8;
    public static double shootTime = 1200;
    public static double waitTime = 1600;
    public static double classifierTime = 1400;
    public static double intakeShootRPM = 1000;
    public static double intakingRPM = 1100;
    public static double stopperDown = 0;
    public static double stopperUp = 0.3;
    public static double hoodUp = 0.53;
    public static double hoodDown = 0.03;
    boolean cameragood;
    public double turretTarget = -2;
    public static double targetoffset = 2;
    public static double flywheelRPM = 2275;
    boolean turretZero = false;


    PrismAnimations.RainbowSnakes rainbow = new PrismAnimations.RainbowSnakes();
    PrismAnimations.Solid solidRed =new PrismAnimations.Solid(Color.RED);
    PrismAnimations.Solid solidPink = new PrismAnimations.Solid(Color.PINK);
    PrismAnimations.Solid solidGreen = new PrismAnimations.Solid(Color.GREEN);
    PathState pathState;
    private double robotX = 0;
    private double robotY = 0;
    public static double targetTurretangle = 0;
    private double robotHeading;
    double baseTarget = 0;
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
        public static double targetGoalX = RED_GOAL_X;
        public static double targetGoalY = RED_GOAL_Y;
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

    private final Pose startPose = new Pose(125.5, 123.1, Math.toRadians(306));
    private final Pose closeZoneShootPose = new Pose (99, 100, Math.toRadians(0));
    private final Pose closeLoadStartPose = new Pose (95, 83.5, Math.toRadians(0));
    private final Pose closeLoadEndPose = new Pose (126, 83.5, Math.toRadians(0));
    private final Pose middleLoadStartPose = new Pose (96, 58, Math.toRadians(0));
    private final Pose middleLoadEndPose = new Pose (133, 58, Math.toRadians(0));
    private final Pose farLoadStartPose = new Pose (95, 34, Math.toRadians(0));
    private final Pose farLoadControlPose = new Pose (100, 55, Math.toRadians(0));
    private final Pose farLoadEndPose = new Pose (133, 34, Math.toRadians(0));
    private final Pose classifierSetup = new Pose (120, 65, Math.toRadians(10));
    private final Pose classifierEmpty = new Pose (127, 71.5, Math.toRadians(10));
    private final Pose leavePose = new Pose (110,88,Math.toRadians(270));

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
                follower.followPath(CloseStartDriveCloseShoot, 1, true);
                flywheel.setVelocity(flywheelRPM);
                rhoodtilt.setPosition(hoodUp);
                stateTimer.reset();
                setPathState(PathState.DRIVE_CLOSESTARTPOS_CLOSESHOOTPOS);
                break;

            case DRIVE_CLOSESTARTPOS_CLOSESHOOTPOS:
                if (!follower.isBusy()) {
                    if (stateTimer.milliseconds() > waitTime) {
                        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solidGreen);
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
                    follower.followPath(CloseShootDriveCloseLoad);
                    stateTimer.reset();
                    setPathState(PathState.DRIVE_CLOSESHOOTPOS_CLOSELOAD);
                }
                break;
            case DRIVE_CLOSESHOOTPOS_CLOSELOAD:
                if (!follower.isBusy()){
                    follower.followPath(CloseLoad, 1, false);
                    intake.setVelocity((intakingRPM * 145.1)/60);
                    stateTimer.reset();
                    setPathState(PathState.CLOSELOAD);
                }
                break;
            case CLOSELOAD:
                if (!follower.isBusy()){
                    intake.setVelocity(0);
                    follower.followPath(DriveCloseLoadCloseShoot, 7, false);
                    stateTimer.reset();
                    setPathState(PathState.DRIVE_CLOSELOADENDPOS_CLOSESHOOTPOS);
                }
                break;
            case DRIVE_CLOSELOADENDPOS_CLOSESHOOTPOS:
                if (!follower.isBusy()){
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
                    follower.followPath(DriveCloseShootMiddleLoad, true);
                    stateTimer.reset();
                    setPathState(PathState.DRIVE_CLOSESHOOTPOS_MIDDLELOAD);
                }
                break;
            case DRIVE_CLOSESHOOTPOS_MIDDLELOAD:
                if (!follower.isBusy()){
                    follower.followPath(MiddleLoad, 0.8, true);
                    intake.setVelocity((intakingRPM * 145.1)/60);
                    stateTimer.reset();
                    setPathState(PathState.MIDDLELOAD);
                }
                break;
            case MIDDLELOAD:
                if (!follower.isBusy()){
                    intake.setVelocity(0);
                    follower.followPath(DriveMiddleLoadClassifierSetup, 0.8, true);
                    stateTimer.reset();
                    setPathState(PathState.CLASSIFIERSETUP);
                }
                break;
            case CLASSIFIERSETUP:
                if (!follower.isBusy()){
                    follower.followPath(ClassifierEmpty, 1, true);
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
                if (!follower.isBusy()) {
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
                    follower.followPath(DriveCloseShootFarLoad, true);
                    stateTimer.reset();
                    setPathState(PathState.DRIVE_CLOSESHOOTPOS_FARLOADSTARTPOS);
                }
                break;
            case DRIVE_CLOSESHOOTPOS_FARLOADSTARTPOS:
                if (!follower.isBusy()){
                    follower.followPath(FarLoad, 1,true);
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
                if (!follower.isBusy()) {
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
                    follower.followPath(FarShootLeave, true);
                    stateTimer.reset();
                    setPathState(PathState.LEAVE);
                }
                break;
            case LEAVE:
                if (!follower.isBusy()){
                    NewBlueGoal.startingPose = follower.getPose();
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
        flywheel = new DualPidMotor (hardwareMap, "bottomflywheel", "topflywheel");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        prism = hardwareMap.get(GoBildaPrismDriver.class,"prism");

        solidRed.setBrightness(100);
        solidRed.setStartIndex(0);
        solidRed.setStopIndex(24);

        solidPink.setBrightness(100);
        solidPink.setStartIndex(0);
        solidPink.setStopIndex(24);

        rainbow.setNumberOfSnakes(3);
        rainbow.setSnakeLength(3);
        rainbow.setSpacingBetween(2);
        rainbow.setSpeed(0.6f);

        rbstop.setPosition(0.3);
        rhoodtilt.setPosition(0);

        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solidRed);
        follower.setStartingPose(startPose);
        robotHeading = follower.getHeading();
        buildPaths();
    }

    public void start() {
        setPathState(PathState.START);
        rbstop.setPosition(stopperDown);
        turret.setTargetAngle(turretFallBackAngle);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solidPink);
        flywheel.setVelocity(flywheelRPM);
        stateTimer.reset();
    }

    @Override
    public void loop() {
        flywheel.Update();
        loopTimer.reset();
        follower.update();
        updatePoseFromPedro();
        statePathUpdate();
        RobotState.savedPose = follower.getPose(); // save every loop

        dt = loopTimer.seconds();
        baseTarget = (calculateTurretAngleFromOdometry());
        turret.setTargetAngle(baseTarget);
        turret.update(dt);
    }


    @Override
    public void stop() {
        prism.clearAllAnimations();
        prism.updateAllAnimations();
        RobotState.savedPose = follower.getPose();
    }


}