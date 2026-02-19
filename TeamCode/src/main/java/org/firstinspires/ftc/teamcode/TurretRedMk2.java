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
@TeleOp(name = "turret red Mk2", group = "Competition")

public class TurretRedMk2 extends OpMode {
    DcMotorEx turretMotor, intake;
    DcMotor frontLeft, frontRight, backLeft, backRight;
    DualPidMotor flywheel;
    Servo rbstop, rhoodtilt;
    Limelight3A limelight;
    public double targetAngle;
    public double turretAngle;
    public double output;

    // --- Compute magnitude ---
    public double deadband = 0.2;
    public static double kP = 0.006;
    public static double kF = 0.01;
    public static double MAX_ANGLE = 120;
    public static double MIN_ANGLE = -90
            ;
    public static double maxPower = 1;
    public static double goaltarget = 0;
    public double lastOutput = 0;
    public static double maxChange = 0.06;
    public double lastGoodtTx = 0;
    private static final double MIN_RPM = 0.0;
    private static final double MAX_RPM = 5800.0;

    private static double MAX_TILT =  0.65;
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
    public static double RPM_AT_2M = 2725.0;
    public static double RPM_AT_FAR = 3050;
    public static double TILT_AT_FAR = 0.6;
    public static double TILT_AT_1M = .34;
    public static double TILT_AT_2M = .55;
    public static double RPM_M, RPM_C, TILT_M, TILT_C;
    long lastTagTime = 0;
    public static double TAG_TIMEOUT_MS = 1000;
    public double targetFlywheelRPM = 0;
    public double lastGoodFlywheelRPM;
    public double lastGoodHoodTilt = 0;
    double error;
    public double baseTarget = 0;
    public static double targetTurretangle = 2; // target angle between the turret and the target in degrees
    public double rx;
    public double usedRPM;
    public double usedTILT;
    public static double intakeIntakingTargetRPM = 450;
    public static double intakeShootingTargetRPM = 1000;
    public static double stopperDown = 0.13;
    public static double stopperUp = 0;
    public double distanceInches;
    public double lastBaseTarget;
    public static double turretAcceptableError = 0.5;
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
    boolean override;
    public double targetHoodTilt = 0;
    @Override
    public void init(){
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        frontRight = hardwareMap.get(DcMotor.class, "frontright");
        frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        backRight = hardwareMap.get(DcMotor.class, "backright");
        backLeft = hardwareMap.get(DcMotor.class, "backleft");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rbstop = hardwareMap.get(Servo.class, "rbstop");
        rhoodtilt = hardwareMap.get(Servo.class, "rhoodtilt");
        flywheel = new DualPidMotor (hardwareMap, "topflywheel", "bottomflywheel");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(1); // Switch to pipeline number 1

        override = false;

        rbstop.setPosition(0);
        rhoodtilt.setPosition(MIN_TILT);
    }

    @Override
    public void start(){
        limelight.start();
    }

    @Override
    public void loop() {
        updateModels();
        flywheel.Update();
        double cx = gamepad2.right_stick_x;
        double cy = -gamepad2.right_stick_y;
        double magnitude = Math.hypot(cx, cy); // magnitude is basically a measurement of how much we are pushing the joystick, as in the hypotenuse of our joystick vector. Not just
        // in one direction. A magnitude of 1 is the stick being pushed exactly forward/backwards or right/left.

        if (!override && gamepad2.dpadUpWasPressed()){
            override = true;
        } else if (override && gamepad2.dpadDownWasPressed()){
            override = false;
        }

        boolean tagRecentlySeen = System.currentTimeMillis() - lastTagTime < TAG_TIMEOUT_MS; // Checks to see if the Limelight has seen the apriltag, goes false after the Limelight hasnt seen anything longer than TAG_TIMEOUT_MS
        boolean trackingready = (tagRecentlySeen || override); // Senses whether or not the tracking system(includes override) is ready, used to failsafe our flywheel
        boolean intaking = (gamepad2.left_trigger > 0.2) && !(gamepad2.right_trigger > 0.2); // Just used to see if we are currently intaking and the flywheel isnt being powered (Mainly for LEDs)
        boolean flywheelspin = ((gamepad2.right_trigger > 0.2) && !(intaking) && trackingready);
        boolean spinningup = ((gamepad2.right_trigger > 0.2) && (flywheel.getCurrentRPM()) < lastGoodFlywheelRPM - 100);
        boolean goodforlaunch = ((trackingready)); // Checks if our flywheel is close enough to target speed and is targeted
        boolean launching = (gamepad2.right_trigger > 0.2) && (gamepad2.left_trigger > 0.2) && (goodforlaunch); // pretty much only used for our LEDs
        boolean unjam = (gamepad2.b); // Only spins the flywheel in reverse
        boolean unload = (gamepad2.x); // Spins flywheel in reverse as well as the intake
        boolean totaloverride = (gamepad2.y); // Used for if our turret rotation breaks or just jams really badly, reverts camera rotation back to the old setup where we rotate the whole bot so that we can still camera-based aim
        boolean turretzero = (gamepad2.a); // Brings turret back to the 0 point
        boolean jammed = ((gamepad2.right_trigger > 0.2) && ((flywheel.getCurrentRPM() < 50) && Math.abs(lastGoodFlywheelRPM) > 1000) && !unjam && !unload); // Primitive jamming function, just senses for whether or not the flywheel is trying to spin but isnt.
        boolean turretjoystick = ((magnitude > 0.8) || override);
        boolean turretcamera = (!turretjoystick && tagRecentlySeen);

        // This whole if (result != null && result.isValid) function does not directly control anything, just calibrates our RPM model, caches those valuses, and updates the lastTagTime(used for our tagRecentlySeen Boolean)
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            lastGoodtTx = result.getTx(); // angle from tag camera-relative(from limelight)

            double distanceMeters = 1.7 / Math.sqrt(result.getTa()); // the constant here is a tuned value, taken by measuring Ta at measured distances and multiplying the distance by the root of the Ta at that distance. THIS IS IN METERS! CALIBRATE IN METERS!
            distanceInches = (distanceMeters * 39.3701); // Converts the distanceMeters into inches so we can feed it into our RPM model

            double lastGoodDistance = distanceInches;

            if (distanceInches < 100) {
                // Clamps outputs to motors and servos to the usable pre-set range from our variables before init, and calculates the output RPM and Tilt
                targetFlywheelRPM = clamp((RPM_M * distanceInches) + RPM_C, MIN_RPM, MAX_RPM);
                targetHoodTilt = clamp((TILT_M * distanceInches) + TILT_C, MIN_TILT, MAX_TILT);
            } else {
                targetFlywheelRPM = RPM_AT_FAR;
                targetHoodTilt = TILT_AT_FAR;
            }

            // Caches last good values from the camera, we use these values for controlling our
            // motors and servos so we can avoid any output dropouts if the camera loses good output
            lastGoodHoodTilt = targetHoodTilt;
            lastGoodFlywheelRPM = targetFlywheelRPM;
            lastTagTime = System.currentTimeMillis();
        }
            // we nest our turret controls inside of the !totaloverride so that if we run totaloverride and aim the bot with the drivetrain, we can easily lock the turret in place

        turretAngle = (turretMotor.getCurrentPosition() / ticksPerTurretRev) * 360.0; // determines our Turret position in degrees from 0(0 is set at initiation, needs to be set exactly forwards or our limits wont work)
        if (launching){
            flywheel.setVelocity(lastGoodFlywheelRPM);
            intake.setVelocity((intakeShootingTargetRPM*145.1)/60);
            rbstop.setPosition(stopperUp);
            rhoodtilt.setPosition(lastGoodHoodTilt);
        } else if (flywheelspin){
            flywheel.setVelocity(lastGoodFlywheelRPM);
            rbstop.setPosition(stopperDown);
            rhoodtilt.setPosition(lastGoodHoodTilt);
        } else if (intaking) {
            intake.setVelocity((intakeIntakingTargetRPM*145.1)/60);
            flywheel.setVelocity(0);
            rbstop.setPosition(stopperDown);
            rhoodtilt.setPosition(MIN_TILT);
        } else if (unjam){
            flywheel.setVelocity(-1500);
            intake.setVelocity(0);
            rbstop.setPosition(stopperUp);
        } else if (unload) {
            intake.setVelocity(-intakeShootingTargetRPM);
            flywheel.setVelocity(-1500);
            rbstop.setPosition(stopperUp);
        } else {
            flywheel.setVelocity(0);
            intake.setVelocity(0);
            rbstop.setPosition(stopperDown);
        }

        if (tagRecentlySeen){
            rhoodtilt.setPosition(lastGoodHoodTilt);
        } else {
            rhoodtilt.setPosition(MIN_TILT);
        }

        if (!totaloverride) {
            // Real meat and potatoes of our logic here. Takes the values from our camera, booleans, and controls and turns them into actual actions.
            if (turretjoystick) { // Runs turret off of joystick if magnitude of joystick exceeds 0.8.
                if (!turretzero) {
                    baseTarget = Math.toDegrees(Math.atan2(cx, cy));
                } else {
                    baseTarget = 0;
                }

                if (!override) {
                    lastGoodFlywheelRPM = targetFlywheelRPM;
                    lastGoodHoodTilt = targetHoodTilt;
                } else {
                    lastGoodFlywheelRPM = RPM_AT_1M;
                    lastGoodHoodTilt = TILT_AT_1M;
                }
                // most of this code advancement is needless and unnecessary as of right now, but when we lengthen our cables out it will allow 360 tracking of the turret with minimal code changes, just changes to the software limits of the turret rotation.
                // This works by creating rotation values +- 360 degrees from the target's orientation from the robot and choosing the closest pathway that is within the limits of our turret.

            } else if (turretcamera) { // Runs off of the camera if the joystick isnt significantly pressed in any direction and a tag is seen.
                if (!turretzero) {
                    baseTarget = turretAngle + targetTurretangle - lastGoodtTx; // Calculates the angle the turret would be at if it were perfectly aligned
                } else {
                    baseTarget = 0;
                }

            }
            lastBaseTarget = baseTarget;
        }
            double bestError = 0;
            double minChange = Double.MAX_VALUE;

            for (int k = -1; k <= 1; k++) { // calculates 3 different equivalent angles based on the current angle from the target, so we can see if any possible rotations to the target are legal
                double candidate = baseTarget + 360 * k;

                if (candidate >= MIN_ANGLE && candidate <= MAX_ANGLE) { // filters out any illegal turret positions that were calculated beforehand (likely 2/3 or all 3 positions generated)
                    double err = candidate - turretAngle; // figures out how far the turret would need to rotate
                    if (Math.abs(err) < minChange) { // chooses the smallest change available out of the selections
                        minChange = Math.abs(err);
                        bestError = err;
                    }
                }
            }

            if (minChange == Double.MAX_VALUE) { // clamps rotation and goes to the closest hard stop of rotation to the tag if no calculated angles are legal
                bestError = (baseTarget > MAX_ANGLE ? MAX_ANGLE : MIN_ANGLE) - turretAngle;
            }

            error = bestError; // assigns our error to the best available error provided by our model to take the closest path possible

            // ---- PID ----
            output = kP * error;

            if (Math.abs(error) > turretAcceptableError) {
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

            telemetry.addData("current angle", turretAngle);
            telemetry.addData("target angle", baseTarget);
            telemetry.addData("output", output);
            telemetry.addData("override", override);
            telemetry.addData("total override", totaloverride);
            telemetry.addData("tracking ready", trackingready);
            telemetry.addData("intaking", intaking);
            telemetry.addData("jammed", jammed);
            telemetry.addData("unjam", unjam);
            telemetry.addData("unload", unload);
            telemetry.addData("goodforlaunch", goodforlaunch);
            telemetry.addData("current flywheel RPM", flywheel.getCurrentRPM());
            telemetry.addData("target RPM", lastGoodFlywheelRPM);
            telemetry.addData("targethoodtilt", lastGoodHoodTilt);
            telemetry.addData("spinning up", spinningup);
            telemetry.addData("spinning", flywheelspin);
            telemetry.addData("launching", launching);
            telemetry.addData("distance from tag", distanceInches);
            telemetry.update();


            // totaloverride basically changes tracking to rotating the whole robot to the apriltag, as we did before
            if (totaloverride && gamepad1.left_bumper && tagRecentlySeen) {
                    baseTarget = lastBaseTarget;
                    double TARGET_YAW = 0.8;
                    double ERROR_YAW = (TARGET_YAW - lastGoodtTx);
                    rx = (ERROR_YAW * kP) * kF;
            } else {
                rx = gamepad1.right_stick_x;
            }

// Reads joystick values for our Mecanum logic
            double y = -gamepad1.left_stick_y;  // Forward is positive
            double x = gamepad1.left_stick_x;  // Strafe

// Mecanum mixing
            double frontLeftPower = y + x + (0.8 * rx);
            double backLeftPower = y - x + (0.8 * rx);
            double frontRightPower = y - x - (0.8 * rx);
            double backRightPower = y + x - (0.8 * rx);

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

    }
}