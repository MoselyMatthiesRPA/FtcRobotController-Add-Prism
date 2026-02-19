package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;

@Configurable
@TeleOp(name = "turret red", group = "Competition")

public class TurretRed extends OpMode {
    DcMotorEx turretMotor, intake, test;
    DualPidMotor flywheel;
    Servo rbstop, rhoodtilt;
    Limelight3A limelight;
    public double targetAngle;
    public double turretAngle;
    public double output;

    // --- Compute magnitude ---
    public double deadband = 0.2;
    public static double kP = 0.005;
    public static double kF = 0.019;
    public static double MAX_ANGLE = 120;
    public static double MIN_ANGLE = -120
            ;
    public static double maxPower = 0.6;
    public static double goaltarget = 0;
    public double lastOutput = 0;
    public static double maxChange = 0.012;
    public double lastGoodtTx = 0;
    private static final double MIN_RPM = 0.0;
    private static final double MAX_RPM = 5800.0;

    private static double MAX_TILT =  0.6;
    private static final double MIN_TILT = 0;

    double ticksPerTurretRev = 537.7 * (200.0 / 86.0);
    private static final double INCHES_PER_METER = 39.3701;

    // --- YOUR CAL POINTS ---
    private static final double RAW_AT_1M_IN = 40.0;
    private static final double RAW_AT_2M_IN = 76.5;

    private static final double TRUE_1M_IN = INCHES_PER_METER;
    private static final double TRUE_2M_IN = 2.0 * INCHES_PER_METER;

    // Linear correction: corrected = A*raw + B, fit to (RAW_AT_1M -> TRUE_1M) and (RAW_AT_2M -> TRUE_2M)
    private static final double DIST_A = (TRUE_2M_IN - TRUE_1M_IN) / (RAW_AT_2M_IN - RAW_AT_1M_IN);
    private static final double DIST_B = TRUE_1M_IN - (DIST_A * RAW_AT_1M_IN);
    public static double RPM_AT_1M = 2300.0;
    public static double RPM_AT_2M = 2650.0;
    public static double TILT_AT_1M = .34;
    public static double TILT_AT_2M = .55;
    public static double RPM_M, RPM_C, TILT_M, TILT_C;
    long lastTagTime = 0;
    public static double TAG_TIMEOUT_MS = 1000;
    public double targetFlywheelRPM = 0;

    public static void updateModels() {
        // RPM model: RPM = M*Distance(in) + C, using (TRUE_1M, RPM_AT_1M) and (TRUE_2M, RPM_AT_2M)
        RPM_M = (RPM_AT_2M - RPM_AT_1M) / (TRUE_2M_IN - TRUE_1M_IN);
        RPM_C = RPM_AT_1M - (RPM_M * TRUE_1M_IN);

        TILT_M = (TILT_AT_2M - TILT_AT_1M) / (TRUE_2M_IN - TRUE_1M_IN);
        TILT_C = TILT_AT_1M - (TILT_M * TRUE_1M_IN);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
    boolean override = false;
    boolean turrettrack = true;
    public double targetHoodTilt = 0;
    @Override
    public void init(){
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        rbstop = hardwareMap.get(Servo.class, " rbstop");
        rhoodtilt = hardwareMap.get(Servo.class, "rhoodtilt");
        flywheel = new DualPidMotor (hardwareMap, "topflywheel", "bottomflywheel");
        test = hardwareMap.get(DcMotorEx.class, "test");
        test.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        test.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(1); // Switch to pipeline number 1
        rbstop.setPosition(0);
    }

    @Override
    public void start(){
        limelight.start();
    }

    @Override
    public void loop() {

        updateModels();
        flywheel.Update();
        double x = gamepad2.right_stick_x;
        double y = -gamepad2.right_stick_y;
        double magnitude = Math.hypot(x, y);

        if (turrettrack && gamepad2.leftBumperWasPressed()){
            turrettrack = false;
        } else if (!turrettrack && gamepad2.rightBumperWasPressed()){
            turrettrack = true;
        }

        boolean tagRecentlySeen = System.currentTimeMillis() - lastTagTime < TAG_TIMEOUT_MS;
        boolean intaking = (gamepad2.left_trigger > 0.2) && !(gamepad2.right_trigger > 0.2);
        boolean trackingready = (tagRecentlySeen || override);
//        boolean goodforlaunch = ((trackingready) && (DualPidMotor.getCurrentRPM() > (targetFlywheelRPM - 100)));
//        boolean launching = (gamepad2.right_trigger > 0.2) && (gamepad2.left_trigger > 0.2) && (goodforlaunch);


        turretAngle = (turretMotor.getCurrentPosition() / ticksPerTurretRev) * 360.0;
        LLResult result = limelight.getLatestResult();
        if (turrettrack) {
            if (result != null && result.isValid()) {
                lastGoodtTx = result.getTx();
                targetAngle = turretAngle + goaltarget - lastGoodtTx;

                double distanceMeters = 1.7 / Math.sqrt(result.getTa());
                double distanceInches = (distanceMeters * 39.3701);

                // Clamps outputs to motors and servos to the usable pre-set range from our variables before init, and calculates the output RPM and Tilt
                targetFlywheelRPM = clamp((RPM_M * distanceInches) + RPM_C, MIN_RPM, MAX_RPM);
                targetHoodTilt = clamp((TILT_M * distanceInches) + TILT_C, MIN_TILT, MAX_TILT);

                // Caches last good values from the camera, we use these values for controlling our
                // motors and servos so we can avoid any output dropouts if the camera loses good output
                double lastGoodDistance = distanceInches;
                double lastGoodFlywheelRPM = targetFlywheelRPM;

                double lastGoodHoodTilt = targetHoodTilt;
                double lastTagTime = System.currentTimeMillis();

                rhoodtilt.setPosition(lastGoodHoodTilt);
            } else {
                turrettrack = false;
            }
        }
        if (!turrettrack){
            if (magnitude > 0.2) {
                targetAngle = Math.toDegrees(Math.atan2(x, y));
                if (targetAngle > MAX_ANGLE){
                    targetAngle = MAX_ANGLE;
                } else if (targetAngle < MIN_ANGLE){
                    targetAngle = MIN_ANGLE;
                }
            }
        }
        // ---- WRAPPED ERROR ----
        double error = normalizeAngle(targetAngle - turretAngle);

        // ---- LIMIT AWARE PATH SELECTION ----
        double proposedAngle = turretAngle + error;

        if (proposedAngle > MAX_ANGLE || proposedAngle < MIN_ANGLE) {
            if (error > 0) {
                error -= 360;
            } else {
                error += 360;
            }
        }

        // ---- PID ----
        output = kP * error;

        if (Math.abs(error) > 1.0) {
            output += Math.signum(error) * kF;
        }

// Clamp max torque
        output = Math.max(-maxPower, Math.min(maxPower, output));

// Slew rate limit

 double delta = output - lastOutput;

        if (delta > maxChange) delta = maxChange;
        if (delta < -maxChange) delta = -maxChange;

        output = lastOutput + delta;
        lastOutput = output;

        turretMotor.setPower(output);

        telemetry.addData("turret angle", turretAngle);
         telemetry.addData("target angle", targetAngle);
        telemetry.addData("output", output);

        if (gamepad2.right_trigger > 0.2){
            test.setVelocity((1000*145.1)/60);
        } else{
            test.setVelocity(0);
        }

        if (gamepad2.left_trigger > 0.2){
            intake.setVelocity((1000*145.1)/60);

        } else{
            intake.setVelocity(0);

        }
    }
    public double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}