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
@Autonomous(name = "Red Far Zone 6", group = "Lawrence")
public class RedFar6 extends OpMode {

    public static double turretFallBackAngle = -65.5;
    public static double shootTime           = 2200;
    public static double waitTime            = 1700;
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
        DRIVE_FARSTARTPOS_FARSHOOTPOS,
        FARSHOOT1,
        DRIVE_FARSHOOTPOS_FARLOADSTARTPOS,
        FARLOAD,
        DRIVE_FARLOADENDPOS_FARSHOOTPOS,
        FARSHOOT3,
        LEAVE
    }

    private final Pose startPose        = new Pose(95,  7.2, Math.toRadians(0));
    private final Pose farZoneShootPose = new Pose(86,  16,  Math.toRadians(0));
    private final Pose farLoadStartPose = new Pose(95,  34,  Math.toRadians(0));
    private final Pose farLoadEndPose   = new Pose(134, 34,  Math.toRadians(0));
    private final Pose farLeavePose     = new Pose(120, 10,  Math.toRadians(0));

    private PathChain driveToShoot, driveToFarLoad, farLoad, driveFarLoadToShoot, driveToLeave;

    private void buildPaths() {
        driveToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, farZoneShootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), farZoneShootPose.getHeading()).build();
        driveToFarLoad = follower.pathBuilder()
                .addPath(new BezierLine(farZoneShootPose, farLoadStartPose))
                .setLinearHeadingInterpolation(farZoneShootPose.getHeading(), farLoadStartPose.getHeading()).build();
        farLoad = follower.pathBuilder()
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
                follower.followPath(driveToShoot, 0.6, true);
                flywheel.setTargetRPM(3075);
                hood.setPosition(hoodUp);
                stateTimer.reset();
                setPathState(PathState.DRIVE_FARSTARTPOS_FARSHOOTPOS);
                break;
            case DRIVE_FARSTARTPOS_FARSHOOTPOS:
                turret.setTargetAngle(turretFallBackAngle);
                if (follower.atPose(farZoneShootPose, 1, 1, 0.05) || stateTimer.milliseconds() > 4000) {
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
                    turret.setTargetAngle(0);
                    flywheel.setTargetRPM(0);
                    follower.followPath(driveToFarLoad);
                    stateTimer.reset();
                    setPathState(PathState.DRIVE_FARSHOOTPOS_FARLOADSTARTPOS);
                }
                break;
            case DRIVE_FARSHOOTPOS_FARLOADSTARTPOS:
                if (!follower.isBusy()) {
                    follower.followPath(farLoad, 0.5, true);
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
                turret.setTargetAngle(turretFallBackAngle);
                if (follower.atPose(farZoneShootPose, 1, 1, 0.05) || stateTimer.milliseconds() > 3000) {
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
