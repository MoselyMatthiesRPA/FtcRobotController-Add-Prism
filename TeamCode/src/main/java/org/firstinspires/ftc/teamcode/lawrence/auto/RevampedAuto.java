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

@Configurable
@Autonomous(name = "Revamped", group = "Lawrence")
public class RevampedAuto extends OpMode {

    // ── Per-auto constants ────────────────────────────────────────────────────
    public static double turretFallBackAngle = 65.5;
    public static double shootTime           = 2200;
    public static double waitTime            = 1700;
    public static double intakeShootRPM      = 800;
    public static double intakingRPM         = 1100;
    public static double stopperDown         = 0.13;
    public static double stopperUp           = 0;
    public static double hoodUp              = 0.75;
    public static double hoodDown            = 0.03;

    // ── Hardware ──────────────────────────────────────────────────────────────
    private Follower follower;
    private Flywheel flywheel;
    private Turret turret;
    private DcMotorEx intake;
    private Servo stopper, hood;
    private Limelight3A limelight;
    private GoBildaPrismDriver prism;

    // ── LED ───────────────────────────────────────────────────────────────────
    private final PrismAnimations.Solid ledBlue = new PrismAnimations.Solid(Color.BLUE);
    private final PrismAnimations.Solid ledPink = new PrismAnimations.Solid(Color.PINK);

    // ── Timing & state ────────────────────────────────────────────────────────
    private Timer pathTimer        = new Timer();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime loopTimer  = new ElapsedTime();
    private double dt = 0;
    private PathState pathState;

    public enum PathState {
        START,
        DRIVE_FARSTARTPOS_FARSHOOTPOS,
        FARSHOOT1,
        LEAVE
    }

    // ── Poses ─────────────────────────────────────────────────────────────────
    private final Pose startPose        = new Pose(49, 7.2, Math.toRadians(180));
    private final Pose farZoneShootPose = new Pose(58, 16,  Math.toRadians(180));
    private final Pose farLeavePose     = new Pose(24, 10,  Math.toRadians(180));

    // ── Paths ─────────────────────────────────────────────────────────────────
    private PathChain driveToShoot, driveToLeave;

    private void buildPaths() {
        driveToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, farZoneShootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), farZoneShootPose.getHeading())
                .build();
        driveToLeave = follower.pathBuilder()
                .addPath(new BezierLine(farZoneShootPose, farLeavePose))
                .setLinearHeadingInterpolation(farZoneShootPose.getHeading(), farLeavePose.getHeading())
                .build();
    }

    private void stateUpdate() {
        switch (pathState) {
            case START:
                if (stateTimer.milliseconds() > 20000) {
                    follower.followPath(driveToShoot, 0.7, true);
                    flywheel.setTargetRPM(3175);
                    hood.setPosition(hoodUp);
                    stateTimer.reset();
                    setPathState(PathState.DRIVE_FARSTARTPOS_FARSHOOTPOS);
                }
                break;

            case DRIVE_FARSTARTPOS_FARSHOOTPOS:
                turret.setTargetAngle(turretFallBackAngle);
                if (follower.atPose(farZoneShootPose, 2, 2, 1) || stateTimer.milliseconds() > 4000) {
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
                    follower.followPath(driveToLeave);
                    stateTimer.reset();
                    setPathState(PathState.LEAVE);
                }
                break;

            case LEAVE:
                if (!follower.isBusy()) {
                    prism.clearAllAnimations();
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
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, ledBlue);

        stopper.setPosition(stopperUp);
        hood.setPosition(hoodDown);

        pathTimer = new Timer();
        buildPaths();
        pathState = PathState.DRIVE_FARSTARTPOS_FARSHOOTPOS;
    }

    @Override
    public void start() {
        stopper.setPosition(Math.min(stopperDown, RobotConstants.STOPPER_DOWN));
        turret.setTargetAngle(turretFallBackAngle);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, ledPink);
        stateTimer.reset();
        setPathState(PathState.START);
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
