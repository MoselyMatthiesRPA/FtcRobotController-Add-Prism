package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

@Configurable
@TeleOp(name = "Red Alliance Teleop", group = "Competition")
public class redOpmode1 extends OpMode {

    // --- HARDWARE AND SERVO VARIABLES ---
    public DcMotorEx intake;
    private DualPidMotor flywheel;
    public DcMotor frontLeft, frontRight, backRight, backLeft;
    public Servo lbstop,rbstop,lhoodtilt, rhoodtilt;
    public final double triggerDeadzone = 0.2;
    public final double hoodDown = 0.02;

    // --- MOTOR VELOCITY CONVERSION ---

    GoBildaPrismDriver prism;
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    PrismAnimations.Solid solidRed = new PrismAnimations.Solid(Color.RED);
    PrismAnimations.Solid solidPink = new PrismAnimations.Solid(Color.PINK);
    PrismAnimations.Solid solidBlue = new PrismAnimations.Solid(Color.BLUE);
    private static final double TICKS_PER_REV = 28.0;
    private static final double VELOCITY_CONVERSION_FACTOR = TICKS_PER_REV / 60.0; // ticks/sec per RPM

    // --- DISTANCE UNITS ---
    private static final double INCHES_PER_METER = 39.3701;

    // --- YOUR CAL POINTS ---
    private static final double RAW_AT_1M_IN = 40.0;
    private static final double RAW_AT_2M_IN = 76.5;

    private static final double TRUE_1M_IN = INCHES_PER_METER;
    private static final double TRUE_2M_IN = 2.0 * INCHES_PER_METER;

    // Linear correction: corrected = A*raw + B, fit to (RAW_AT_1M -> TRUE_1M) and (RAW_AT_2M -> TRUE_2M)
    private static final double DIST_A = (TRUE_2M_IN - TRUE_1M_IN) / (RAW_AT_2M_IN - RAW_AT_1M_IN);
    private static final double DIST_B = TRUE_1M_IN - (DIST_A * RAW_AT_1M_IN);

    // --- RPM TARGETS (HARDCODED) ---
    public static double RPM_AT_1M = 2300.0;
    public static double RPM_AT_2M = 2650.0;
    public static double TILT_AT_1M = .34;
    public static double TILT_AT_2M = .55;
    public static double RPM_M, RPM_C, TILT_M, TILT_C;

    public static void updateModels() {
        // RPM model: RPM = M*Distance(in) + C, using (TRUE_1M, RPM_AT_1M) and (TRUE_2M, RPM_AT_2M)
        RPM_M = (RPM_AT_2M - RPM_AT_1M) / (TRUE_2M_IN - TRUE_1M_IN);
        RPM_C = RPM_AT_1M - (RPM_M * TRUE_1M_IN);

        TILT_M = (TILT_AT_2M - TILT_AT_1M) / (TRUE_2M_IN - TRUE_1M_IN);
        TILT_C = TILT_AT_1M - (TILT_M * TRUE_1M_IN);
    }

    // Safety clamp
    private static final double MIN_RPM = 0.0;
    private static final double MAX_RPM = 5800.0;

    private static final double MAX_TILT =  0.55;
    private static final double MIN_TILT = 0.01;

    // --- LIVE DATA ---
    private double yawDegrees = 0.8;
    private double distanceMeters = 0.0;
    private double targetFlywheelRPM = 0.0;
    private double targetHoodTilt = 0.0;
    private double lastLoopTime = 0.0;

    // --- CACHED VALUES ---

    double lastGoodDistance = 0.0;
    double lastGoodFlywheelRPM = 0.0;
    double lastGoodHoodTilt = 0.02;
    // we are not storing a starting value for lastGoodYaw since it is so situation-dependent
    double lastGoodYaw;
    public static double TARGET_YAW = 0.7;  // 0.8 starting is good
    double ERROR_YAW;

    double LAST_ERROR_YAW;


    public static double intakeTargetRPM = 450;
    double ReverseIntake = -500;

    public boolean launchOverride;
    public double rx;
    public static double kP = 0.024;
    public static double kD = 0.01;
    public static double kF = 1.2;
    public static double turnMultiplier = 0.1;

    long lastTagTime = 0;
    static final long TAG_TIMEOUT_MS = 1000;

    Limelight3A limelight;

    @Override
    public void init() {

        // --- HARDWARE INITIALIZATION ---
        frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        frontRight = hardwareMap.get(DcMotor.class, "frontright");
        backLeft = hardwareMap.get(DcMotor.class, "backleft");
        backRight = hardwareMap.get(DcMotor.class, "backright");
        flywheel = new DualPidMotor (hardwareMap, "topflywheel", "bottomflywheel");
        rhoodtilt = hardwareMap.get(Servo.class, "rhoodtilt");
        lhoodtilt = hardwareMap.get(Servo.class, "lhoodtilt");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        lbstop = hardwareMap.get(Servo.class, "lbstop");
        rbstop = hardwareMap.get(Servo.class, "rbstop");
        prism = hardwareMap.get(GoBildaPrismDriver.class,"prism");
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(16, 124, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        // --- MOTOR SETUP ---
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        rbstop.setDirection(Servo.Direction.REVERSE);
        lhoodtilt.setDirection(Servo.Direction.REVERSE);

        // --- SERVO SETUP ---
        lhoodtilt.setPosition(MIN_TILT);
        rhoodtilt.setPosition(MIN_TILT);
        rbstop.setPosition(0);
        lbstop.setPosition(0);

        // --- VISION SETUP ---

        launchOverride = false;
        updateModels();

        solidRed.setBrightness(100);
        solidRed.setStartIndex(0);
        solidRed.setStopIndex(36);

        solidPink.setBrightness(100);
        solidPink.setStartIndex(12);
        solidPink.setStopIndex(36);

        solidBlue.setBrightness(100);
        solidBlue.setStartIndex(0);
        solidBlue.setStopIndex(11);


        prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, solidRed);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(1); // Switch to pipeline number 1
    }

    @Override
    public void start(){
        prism.clearAllAnimations();
        prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, solidPink);
        prism.insertAndUpdateAnimation(LayerHeight.LAYER_1, solidBlue);
        limelight.start();
    }

    @Override
    public void loop() {

        // -------------------------------
        // 1) CAMERA CONTROLS AND OUTPUTS
        // -------------------------------
        updateModels();
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            distanceMeters = 1.7/Math.sqrt(result.getTa());
            double distanceInches = (distanceMeters * 39.3701);
            yawDegrees =  result.getTx();

            // Clamps outputs to motors and servos to the usable pre-set range from our variables before init, and calculates the output RPM and Tilt
            targetFlywheelRPM = clamp((RPM_M * distanceInches) + RPM_C, MIN_RPM, MAX_RPM);
            targetHoodTilt = clamp((TILT_M * distanceInches) + TILT_C, MIN_TILT, MAX_TILT);

            // Caches last good values from the camera, we use these values for controlling our
            // motors and servos so we can avoid any output dropouts if the camera loses good output
            lastGoodDistance = distanceInches;
            lastGoodFlywheelRPM = targetFlywheelRPM;

            lastGoodHoodTilt = targetHoodTilt;
            lastGoodYaw = result.getTx();
            lastTagTime = System.currentTimeMillis();
        }

        // -------------------------------f
        // 2) CONTROLS AND BOOLEAN LOGIC
        // -------------------------------

        if (gamepad2.dpadUpWasPressed()) {
            launchOverride = true;
        } else if (gamepad2.dpadDownWasPressed()) {
            launchOverride = false;
        }
        // boolean logic that tells us if the camera has seen an april tag since the timeout setting
        boolean tagRecentlySeen = System.currentTimeMillis() - lastTagTime < TAG_TIMEOUT_MS;
        // This goodForLaunch boolean basically just goes positive if the right trigger is held
        // and a tag is seen, signifying that we are ready to launch.
        boolean goodForLaunch = (gamepad2.right_trigger > triggerDeadzone) && tagRecentlySeen;

        // Combination of controller input and camera output for spinning up motors and setting servo positions
        // based on the last good readings, only works if tag has been seen recently(will code a backup for this to continue functionality)
        if (gamepad2.right_trigger > 0.2 && launchOverride){
            targetFlywheelRPM = RPM_AT_1M;
            flywheel.setVelocity(targetFlywheelRPM);
            lhoodtilt.setPosition(TILT_AT_1M);
            rhoodtilt.setPosition(TILT_AT_1M);
        } if (goodForLaunch) {
            flywheel.setVelocity(lastGoodFlywheelRPM);
            lhoodtilt.setPosition(lastGoodHoodTilt);
            rhoodtilt.setPosition(lastGoodHoodTilt);
            // Shuts off motors when the trigger isn't pulled, but keeps the hood adjusting to keep ready to fire(mainly cause it looks cool)
        } else if ((gamepad2.right_trigger < triggerDeadzone) && tagRecentlySeen) {
            flywheel.setVelocity(1);
            lhoodtilt.setPosition(lastGoodHoodTilt);
            rhoodtilt.setPosition(lastGoodHoodTilt);
            // fully shuts down the motors and drops the hood if the april tag has not been seen for too long, under the tagRecentlySeen function used in goodForLaunch
        } else {
            flywheel.setVelocity(0);
            lhoodtilt.setPosition(hoodDown);
            rhoodtilt.setPosition(hoodDown);
        }

        if (gamepad2.left_trigger > 0.2){
            lbstop.setPosition(0);
            rbstop.setPosition(0);
            intake.setVelocity((intakeTargetRPM * 145.1)/60);
        } else if (gamepad2.left_trigger < 0.2 && gamepad2.y) {
            intake.setVelocity((ReverseIntake * 145.1) / 60);
            flywheel.setVelocity(-2000);
            rbstop.setPosition(0);
            lbstop.setPosition(0);
        } else if (gamepad2.left_trigger < 0.2) {
            intake.setVelocity(0);
            lbstop.setPosition(0.15);
            rbstop.setPosition(0.15);
        }
        flywheel.Update();

        // -------------------------------
        // 3) Drivetrain Control
        // -------------------------------

// rx is stored seperately in this function, while being declared a double in the declarations section of our code
// so as to allow us to assign rx to be whatever we want, allowing this override function that uses the camera to
// align the robot to the april tag on the goal.

        if (gamepad1.left_bumper && tagRecentlySeen){
            TARGET_YAW = 0.8;
            ERROR_YAW = (TARGET_YAW - yawDegrees);
            rx = (ERROR_YAW * kP) - kD*(ERROR_YAW - LAST_ERROR_YAW) * kF;
            LAST_ERROR_YAW = ERROR_YAW;

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

        double currentTime = getRuntime();
        double deltaTime = currentTime - lastLoopTime;
        lastLoopTime = currentTime;

//        if (gamepad2.left_trigger > 0.2) {
//            intake.setVelocity((intakeTargetRPM * 145.1) / 60);
//            rbstop.setPosition(0);
//            lbstop.setPosition(0);
//        } else {
//            intake.setVelocity(0);
//            rbstop.setPosition(0.15);
//            lbstop.setPosition(0.15);
//        }
        flywheel.Update();
        // -------------------------------
        // 3) TELEMETRY
        // -------------------------------
        telemetry.addData("--- AUTO SHOOTER V6 ---", "");
        telemetry.addData("Target RPM", "%.0f", lastGoodFlywheelRPM);
        telemetry.addData("Target Hood Tilt", "%.1f", targetHoodTilt);
        telemetry.addData("Flywheels", ((gamepad2.right_trigger > 0.2) && tagRecentlySeen) ? "ON (B)" : "OFF");
        telemetry.addData("Last Loop Time", deltaTime);
        telemetry.addData("Tag Recent", tagRecentlySeen);
        telemetry.addData("Distance", lastGoodDistance);
        telemetry.addData("Yaw(Degrees)", yawDegrees);
        telemetry.addData("Launch Override", (launchOverride) ? "ON (B)" : "OFF");
        telemetry.addData("rpm at 1 meter", RPM_AT_1M);
        telemetry.addData("rpm at 2 meter", RPM_AT_2M);
        telemetry.update();

        if (gamepad2.rightBumperWasPressed()){
            RPM_AT_1M += 50;
        } else if (gamepad2.leftBumperWasPressed()) {
            RPM_AT_1M -= 50;
        } else if (gamepad2.dpadRightWasPressed()){
            RPM_AT_2M += 10;
        } else if (gamepad2.dpadLeftWasPressed()){
            RPM_AT_2M -= 10;
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        prism.clearAllAnimations();
        prism.updateAllAnimations();
    }
    // logic behind our clamp calls, constrains output between our high and low maxes we set when we call clamp
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
