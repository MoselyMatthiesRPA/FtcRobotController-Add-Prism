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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;

@Configurable
@TeleOp(name = "turret red", group = "Competition")

public class TurretRed extends OpMode {
    DcMotorEx turretMotor;
    Limelight3A limelight;
    public double targetAngle;
    public double turretAngle;
    public double output;

    // --- Compute magnitude ---
    public double deadband = 0.2;
    public static double kP = 0.008;
    public static double kF = 0.012;
    public static double MAX_ANGLE = 175;
    public static double MIN_ANGLE = -175;
    public static double maxPower = 0.6;
    public static double goaltarget = 0;
    public double lastOutput = 0;
    public static double maxChange = 0.017;
    public double lastGoodtTx = 0;

    double ticksPerTurretRev = 537.7 * (200.0 / 86.0);

    @Override
    public void init(){
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(1); // Switch to pipeline number 1

    }

    @Override
    public void start(){
        limelight.start();
    }

    @Override
    public void loop() {

        double x = gamepad2.right_stick_x;
        double y = -gamepad2.right_stick_y;
        double magnitude = Math.hypot(x, y);

        turretAngle = (turretMotor.getCurrentPosition() / ticksPerTurretRev) * 360.0;
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            lastGoodtTx = result.getTx();
            targetAngle = turretAngle + goaltarget - lastGoodtTx;
        } else {
            if (magnitude > 0.2) {
                targetAngle = Math.toDegrees(Math.atan2(x, y));
                if (targetAngle > MAX_ANGLE){
                    targetAngle = MAX_ANGLE-5;
                } else if (targetAngle < MIN_ANGLE){
                    targetAngle = MIN_ANGLE+5;
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
    }
    public double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}