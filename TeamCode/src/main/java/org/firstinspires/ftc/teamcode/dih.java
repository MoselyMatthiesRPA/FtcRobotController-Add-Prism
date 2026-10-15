package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.lynx.LynxModule;
import java.util.List;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.PedroCoordinates;
import androidx.annotation.Nullable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;
@Configurable
@TeleOp(name = "dih", group = "Competition")
public class dih extends OpMode {
    public double targetAngle;
        public double turretAngle;
    public double output;

    // --- Compute magnitude ---
    public double deadband = 0.2;
    public static double kP = 0.01;
    public static double kD = 0.0007;
    public static double kF = 0.01;
    public static double MAX_ANGLE = 120;
    public static double MIN_ANGLE = -90;
    public static double maxPower = 1;
    private double robotX = 0;
        private double robotY = 0;
    public static final double ROBOT_RADIUS = 9;
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();
        private double robotHeading = (3.14/2.0); // radians
        public double baseTarget = 0;
        public double lastBaseTarget = 0;
        ElapsedTime loopTimer = new ElapsedTime();


//    DualPidMotor flywheel;
    DcMotor frontLeft, frontRight, backLeft, backRight;
    DcMotorEx flywheel;
    Servo hood;
    Turret turret;

    public double hoodangle = 0;
    public double targetFlywheelRPM = 0;
    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "topflywheel");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
//                flywheel = new DualPidMotor (hardwareMap, "topflywheel");
        frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        frontRight = hardwareMap.get(DcMotor.class, "frontright");
        backLeft = hardwareMap.get(DcMotor.class, "backleft");
        backRight = hardwareMap.get(DcMotor.class, "backright");
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                turret = new Turret(hardwareMap, true);


        hood = hardwareMap.get(Servo.class, "hood");
        hood.setPosition(0);
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();

        if (gamepad1.aWasPressed()) {
            targetFlywheelRPM+= 50;
        } else if (gamepad1.bWasPressed()){
            targetFlywheelRPM -=50;
        }
        if (gamepad1.dpadUpWasPressed()){
            hoodangle += 0.02;
            hood.setPosition(hoodangle);
        } else if (gamepad1.dpadDownWasPressed()){
            hoodangle -= 0.02;
            hood.setPosition(hoodangle);
        }

        double cx = gamepad2.right_stick_x;
        double cy = -gamepad2.right_stick_y;
        double magnitude = Math.hypot(cx, cy); // magnitude is basically a measurement of how much we are pushing the joystick, as in the hypotenuse of our joystick vector. Not just
        // in one direction. A magnitude of 1 is the stick being pushed exactly forward/backwards or right/left.

        boolean turretjoystick = ((magnitude > 0.5));

        if (turretjoystick) {
            // Manual joystick control
            baseTarget = Math.toDegrees(Math.atan2(cx, cy));
        }
        turretAngle = turret.getCurrentAngle(); // determines our Turret position in degrees from 0(0 is set at initiation, needs to be set exactly forwards or our limits wont work)
//        if (gamepad2.dpadRightWasPressed()){
//            baseTarget += 0.5;
//        } else if (gamepad2.dpadLeftWasPressed()){
//            baseTarget -= 0.5;
//        }
        lastBaseTarget = baseTarget;

        // Reads joystick values for our Mecanum logic
        double rx = gamepad1.right_stick_x;
        double y = -gamepad1.left_stick_y;  // Forward is positive
        double x = gamepad1.left_stick_x;  // Strafe

// Mecanum mixing
        double frontLeftPower = y + x + (1 * rx);
        double backLeftPower = y - x + (1 * rx);
        double frontRightPower = y - x - (1 * rx);
        double backRightPower = y + x - (1 * rx);

// Normalize powers so no value exceeds 1.0
        double max = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                Math.max(Math.abs(frontRightPower), Math.abs(backRightPower)));

        if (max > 1.0) {
            frontLeftPower /= max;
            backLeftPower /= max;
            frontRightPower /= max;
            backRightPower /= max;
        }

// Send power to motors
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

        telemetry.addData("targetRPM", targetFlywheelRPM);
        telemetry.addData("hood tilt", hoodangle);
        telemetry.addData("baseTarget", baseTarget);
        telemetry.addData("current angle", turretAngle);
        telemetry.addData("target angle", turret.getTargetAngle());

        telemetry.update();
        flywheel.setVelocity(targetFlywheelRPM);
//        flywheel.setVelocity(targetFlywheelRPM);
//        flywheel.Update();

        turret.setTargetAngle(baseTarget);
        turret.update(dt);
    }
}