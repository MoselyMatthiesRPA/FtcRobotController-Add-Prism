//package org.firstinspires.ftc.teamcode.Autos;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.DualPidMotor;
//import org.firstinspires.ftc.teamcode.NewRedGoal;
//import org.firstinspires.ftc.teamcode.Prism.Color;
//import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
//import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
//import org.firstinspires.ftc.teamcode.RobotState;
//import org.firstinspires.ftc.teamcode.Turret;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//@Configurable
//@Autonomous(name = "Blue far zone 6", group = "Competition")
//public class BlueFar6 extends OpMode {
//    private Follower follower; // Pedro Pathing follower instance
//    private Timer pathTimer;
//    ElapsedTime stateTimer = new ElapsedTime();
//    GoBildaPrismDriver prism;
//    DcMotorEx intake;
//    Turret turret;
//    Servo rbstop, rhoodtilt;
//    Limelight3A limelight;
//    DualPidMotor flywheel;
//    ElapsedTime loopTimer = new ElapsedTime();
//    double dt = 0;
//    public static double turretFallBackAngle = 65.5;
//    public static double shootTime = 2200;
//    public static double waitTime = 1700;
//    public static double classifierTime = 2000;
//    public static double intakeShootRPM = 800;
//    public static double intakingRPM = 1100;
//    public static double stopperDown = 0;
//    public static double stopperUp = 0.3;
//    public static double hoodUp = 0.65;
//    public static double hoodDown = 0.03;
//    boolean cameragood;
//    public double turretTarget = 2;
//    public static double targetoffset = 2;
//    private double robotX = 0;
//    private double robotY = 0;
//    public static double targetTurretangle = 0;
//    private double robotHeading;
//    double baseTarget = 0;
//    public static double flywheelRPM = 3100;
//    private double calculateTurretAngleFromOdometry() {
//        // Get vector from robot to goal (field coordinates)
//        double deltaX = FieldPositions.targetGoalX - robotX;
//        double deltaY = FieldPositions.targetGoalY - robotY;
//
//        // Calculate field-centric angle to goal (in radians)
//        double fieldCentricAngle = Math.atan2(deltaY, deltaX);
//
//        // Convert to robot-centric by subtracting robot's heading
//        double robotCentricAngle = -(fieldCentricAngle - robotHeading);
//
//        // Convert to degrees
//        double angleDegrees = Math.toDegrees(robotCentricAngle);
//
//        // Normalize to [-180, 180] range
//        while (angleDegrees > 180) angleDegrees -= 360;
//        while (angleDegrees < -180) angleDegrees += 360;
//
//        return angleDegrees+targetTurretangle;
//    }
//    public static class FieldPositions {
//        // Define goal positions in inches (Pedro's coordinate system)
//        // Adjust these based on your field's coordinate system
//        public static final double RED_GOAL_X = 130.3727;
//        public static final double RED_GOAL_Y = 127.6425;
//
//        public static final double BLUE_GOAL_X = 13.6273;
//        public static final double BLUE_GOAL_Y = 127.6425;
//
//        // Set based on alliance
//        public static double targetGoalX = BLUE_GOAL_X;
//        public static double targetGoalY = BLUE_GOAL_Y;
//    }
//    private void updatePoseFromPedro() {
//        Pose currentPose = follower.getPose();
//        robotX = currentPose.getX(); // Left is Positive
//        robotY = currentPose.getY(); // Forward is Positive
//        robotHeading = currentPose.getHeading(); // In Radians
//    }
//
//    PrismAnimations.RainbowSnakes rainbow = new PrismAnimations.RainbowSnakes();
//    PrismAnimations.Solid solidBlue =new PrismAnimations.Solid(Color.BLUE);
//    PrismAnimations.Solid solidPink = new PrismAnimations.Solid(Color.PINK);
//    PrismAnimations.Solid solidGreen = new PrismAnimations.Solid(Color.GREEN);
//    PathState pathState;
//    public enum PathState{
//        START,
//        DRIVE_FARSTARTPOS_FARSHOOTPOS,
//        FARSHOOT1,
//        DRIVE_FARSHOOTPOS_MIDDLELOAD,
//        MIDDLELOAD,
//        CLASSIFIERSETUP,
//        CLASSIFIEREMPTY,
//        DRIVE_CLASSIFIEREMPTYPOS_FARSHOOTPOS,
//        FARSHOOT2,
//        DRIVE_FARSHOOTPOS_FARLOADSTARTPOS,
//        FARLOAD,
//        DRIVE_FARLOADENDPOS_FARSHOOTPOS,
//        FARSHOOT3,
//        LEAVE
//    }
//
//    private final Pose startPose = new Pose(49, 6.25, Math.toRadians(180));
//    private final Pose farZoneShootPose = new Pose (57, 15, Math.toRadians(180));
//    private final Pose middleLoadStartPose = new Pose (48, 58, Math.toRadians(180));
//    private final Pose middleLoadEndPose = new Pose (10, 58, Math.toRadians(180));
//    private final Pose farLoadStartPose = new Pose (58, 35, Math.toRadians(180));
//    private final Pose middleLoadControlPose = new Pose (48, 58, Math.toRadians(180));
//    private final Pose farLoadEndPose = new Pose (10, 35, Math.toRadians(180));
//    private final Pose classifierSetup = new Pose (24, 65, Math.toRadians(170));
//    private final Pose classifierEmpty = new Pose (16, 71.5, Math.toRadians(170));
//    private final Pose farLeavePose = new Pose (24, 10, Math.toRadians(180));
//
//    private PathChain FarStartDriveFarShoot, FarShootDriveMiddleLoad, MiddleLoad, DriveMiddleLoadClassifierSetup, ClassifierEmpty, DriveClassifierEmptyFarShoot, DriveFarShootFarLoad, FarLoad, DriveFarLoadFarShoot, FarShootLeave;
//    public void buildPaths() {
//        FarStartDriveFarShoot = follower.pathBuilder()
//                .addPath(new BezierLine(startPose, farZoneShootPose))
//                .setLinearHeadingInterpolation(startPose.getHeading(), farZoneShootPose.getHeading())
//                .build();
//        FarShootDriveMiddleLoad = follower.pathBuilder()
//                .addPath(new BezierLine(farZoneShootPose, middleLoadStartPose))
//                .setLinearHeadingInterpolation(farZoneShootPose.getHeading(), middleLoadStartPose.getHeading())
//                .build();
//        MiddleLoad = follower.pathBuilder()
//                .addPath(new BezierLine(middleLoadStartPose, middleLoadEndPose))
//                .setLinearHeadingInterpolation(middleLoadStartPose.getHeading(), middleLoadEndPose.getHeading())
//                .build();
//        DriveMiddleLoadClassifierSetup = follower.pathBuilder()
//                .addPath(new BezierLine(middleLoadEndPose, classifierSetup))
//                .setLinearHeadingInterpolation(middleLoadEndPose.getHeading(), classifierSetup.getHeading())
//                .build();
//        ClassifierEmpty = follower.pathBuilder()
//                .addPath(new BezierLine(classifierSetup, classifierEmpty))
//                .setLinearHeadingInterpolation(classifierSetup.getHeading(), classifierEmpty.getHeading())
//                .build();
//        DriveClassifierEmptyFarShoot = follower.pathBuilder()
//                .addPath(new BezierCurve(classifierEmpty, middleLoadControlPose, farZoneShootPose))
//                .setLinearHeadingInterpolation(classifierEmpty.getHeading(), farZoneShootPose.getHeading())
//                .build();
//        DriveFarShootFarLoad = follower.pathBuilder()
//                .addPath(new BezierLine(farZoneShootPose, farLoadStartPose))
//                .setLinearHeadingInterpolation(farZoneShootPose.getHeading(), farLoadStartPose.getHeading())
//                .build();
//        FarLoad = follower.pathBuilder()
//                .addPath(new BezierLine(farLoadStartPose, farLoadEndPose))
//                .setLinearHeadingInterpolation(farLoadStartPose.getHeading(), farLoadEndPose.getHeading())
//                .build();
//        DriveFarLoadFarShoot = follower.pathBuilder()
//                .addPath(new BezierLine(farLoadEndPose, farZoneShootPose))
//                .setLinearHeadingInterpolation(farLoadEndPose.getHeading(), farZoneShootPose.getHeading())
//                .build();
//        FarShootLeave = follower.pathBuilder()
//                .addPath(new BezierLine(farZoneShootPose, farLeavePose))
//                .setLinearHeadingInterpolation(farZoneShootPose.getHeading(), farLeavePose.getHeading())
//                .build();
//    }
//    public void statePathUpdate(){
//        switch(pathState){
//            case START:
//                follower.followPath(FarStartDriveFarShoot, 0.9, true);
//                rhoodtilt.setPosition(hoodUp);
//                stateTimer.reset();
//                setPathState(PathState.DRIVE_FARSTARTPOS_FARSHOOTPOS);
//                break;
//
//            case DRIVE_FARSTARTPOS_FARSHOOTPOS:
//                if (!follower.isBusy()){
//                    if (stateTimer.milliseconds() > waitTime) {
//                        stateTimer.reset();
//                        setPathState(PathState.FARSHOOT1);
//                    }
//                }
//                break;
//
//            case FARSHOOT1:
//                if (stateTimer.milliseconds() < shootTime){
//                    rbstop.setPosition(stopperUp);
//                    intake.setVelocity((intakeShootRPM * 145.1)/60);
//                } else if (stateTimer.milliseconds() > shootTime){
//                    intake.setVelocity(0);
//                    rbstop.setPosition(stopperDown);
//                    rhoodtilt.setPosition(hoodDown);
//                    follower.followPath(DriveFarShootFarLoad);
//                    stateTimer.reset();
//                    setPathState(PathState.DRIVE_FARSHOOTPOS_FARLOADSTARTPOS);
//                }
//
//                break;
//            case DRIVE_FARSHOOTPOS_FARLOADSTARTPOS:
//                if (!follower.isBusy()){
//                    follower.followPath(FarLoad, 1, false);
//                    intake.setVelocity((intakingRPM * 145.1)/60);
//                    stateTimer.reset();
//                    setPathState(PathState.FARLOAD);
//                }
//                break;
//            case FARLOAD:
//                if (!follower.isBusy()){
//                    intake.setVelocity(0);
//                    follower.followPath(DriveFarLoadFarShoot, true);
//                    stateTimer.reset();
//                    setPathState(PathState.DRIVE_FARLOADENDPOS_FARSHOOTPOS);
//                }
//                break;
//            case DRIVE_FARLOADENDPOS_FARSHOOTPOS:
//                rhoodtilt.setPosition(hoodUp);
//                if (!follower.isBusy()){
//                    stateTimer.reset();
//                    setPathState(PathState.FARSHOOT3);
//                }
//                break;
//            case FARSHOOT3:
//                if (stateTimer.milliseconds() < shootTime){
//                    rbstop.setPosition(stopperUp);
//                    intake.setVelocity((intakeShootRPM * 145.1)/60);
//                } else if (stateTimer.milliseconds() > shootTime){
//                    intake.setVelocity(0);
//                    rhoodtilt.setPosition(0);
//                    rbstop.setPosition(stopperDown);
//                    follower.followPath(FarShootLeave, true);
//                    stateTimer.reset();
//                    setPathState(PathState.LEAVE);
//                }
//                break;
//            case LEAVE:
//                if (!follower.isBusy()){
//                }
//            default:
//                break;
//        }
//    }
//
//    public void setPathState(PathState newState){
//        pathState = newState;
//        pathTimer.resetTimer();
//    }
//
//    @Override
//    public void init() {
//        pathState = PathState.DRIVE_FARSTARTPOS_FARSHOOTPOS;
//        pathTimer = new Timer();
//        follower = Constants.createFollower(hardwareMap);
//        intake = hardwareMap.get(DcMotorEx.class, "intake");
//        turret = new Turret(hardwareMap, true);
//        rbstop = hardwareMap.get(Servo.class, "rbstop");
//        rhoodtilt = hardwareMap.get(Servo.class, "rhoodtilt");
//        rhoodtilt.setDirection(Servo.Direction.REVERSE);
//        flywheel = new DualPidMotor (hardwareMap, "bottomflywheel", "topflywheel");
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        prism = hardwareMap.get(GoBildaPrismDriver.class,"prism");
//        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
//        limelight.pipelineSwitch(0); // Switch to pipeline number 1
//
//        rbstop.setPosition(stopperUp);
//        rhoodtilt.setPosition(hoodUp);
//        solidBlue.setBrightness(100);
//        solidBlue.setStartIndex(0);
//        solidBlue.setStopIndex(36);
//
//        solidPink.setBrightness(100);
//        solidPink.setStartIndex(0);
//        solidPink.setStopIndex(36);
//
//        rainbow.setNumberOfSnakes(3);
//        rainbow.setSnakeLength(3);
//        rainbow.setSpacingBetween(2);
//        rainbow.setSpeed(0.6f);
//
//        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solidBlue);
//        follower.setStartingPose(startPose);
//        buildPaths();
//    }
//
//    public void start() {
//        setPathState(PathState.START);
//        rbstop.setPosition(stopperDown);
//        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solidPink);
//        flywheel.setVelocity(flywheelRPM);
//        stateTimer.reset();
//    }
//
//    @Override
//    public void loop() {
//        flywheel.Update();
//        loopTimer.reset();
//        follower.update();
//        updatePoseFromPedro();
//        statePathUpdate();
//        RobotState.savedPose = follower.getPose(); // save every loop
//
//        dt = loopTimer.seconds();
//        baseTarget = (calculateTurretAngleFromOdometry());
//        turret.setTargetAngle(baseTarget);
//        turret.update(dt);
//    }
//
//
//    @Override
//    public void stop() {
//        prism.clearAllAnimations();
//        prism.updateAllAnimations();
//    }
//
//}
