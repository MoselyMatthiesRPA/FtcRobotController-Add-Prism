package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Configurable
public class Turret {
    DcMotorEx turretMotor;

    // --- Target State ---
    public static double targetAngleDeg = 0;   // what we want
    private double currentAngleDeg = 0.0;  // what we are at

    // --- PID State ---
    private double lastError = 0.0;
    private double lastOutput = 0.0;

    // --- Constants (tune these) ---
    public static double kP = 0.01;
    public static double kD = 0.001;
    public static double kF = 0.06;
    public static double MAX_ANGLE = 120;
    public static double MIN_ANGLE = -90;
    double ticksPerTurretRev = 537.7 * (200.0 / 87.0);

    public static double maxPower = 1;
    public static double turretAcceptableError = 0.5;
    public static double maxChange = 0.05;
    public static double maxchangescaler = 2.0;
    public double output;

    public Turret(HardwareMap hardwareMap, boolean resetEncoder) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        if (resetEncoder) {
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        turretMotor.setDirection(DcMotorEx.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }



    // ===============================
    // PUBLIC METHODS (used by OpMode)
    // ===============================

    public void setTargetAngle(double baseTarget) {
        this.targetAngleDeg = baseTarget;
    }

    public double getTargetAngle() {
        return targetAngleDeg;
    }

    public double getOutput() {
        return output;
    }

    public double getCurrentAngle() {
        double turretAngle = currentAngleDeg;
        return turretAngle;
    }

    public void update(double dt) {
        // Get current angle from encoder
        currentAngleDeg = getAngleFromEncoder();

        // ---- Equivalent angle selection logic ----
        double minChange = Double.MAX_VALUE;
        double bestError = 0;

        for (int k = -1; k <= 1; k++) {
            double candidate = targetAngleDeg + 360 * k;

            if (candidate >= MIN_ANGLE && candidate <= MAX_ANGLE) {
                double err = candidate - currentAngleDeg;

                if (Math.abs(err) < minChange) {
                    minChange = Math.abs(err);
                    bestError = err;
                }
            }
        }

        if (minChange == Double.MAX_VALUE) {
            bestError =
                    (targetAngleDeg > MAX_ANGLE ? MAX_ANGLE : MIN_ANGLE)
                            - currentAngleDeg;
        }

        double error = bestError;

        // ---- PID ----
        double derivative = (error - lastError) / dt;
        output = kP * error + kD * derivative;

        lastError = error;

        if (Math.abs(error) > turretAcceptableError) {
            output += Math.signum(error) * kF;
        }

        // Clamp max torque
        output = Math.max(-maxPower, Math.min(maxPower, output));

        // ---- Slew rate limiting ----
        double absError = Math.abs(error);

        double maxChangeActive;
        if (absError < 5) {
            maxChangeActive = maxChange * maxchangescaler;
        } else {
            maxChangeActive = maxChange;
        }

        double delta = output - lastOutput;

        if (delta > maxChangeActive) delta = maxChangeActive;
        if (delta < -maxChangeActive) delta = -maxChangeActive;

        output = lastOutput + delta;
        lastOutput = output;

        turretMotor.setPower(output);
    }

    // ===============================
    // PRIVATE METHODS
    // ===============================

    private double getAngleFromEncoder() {
        // Replace with your ticks → degrees conversion
        double ticks = turretMotor.getCurrentPosition();
        return (ticks / ticksPerTurretRev) * 360.0;
    }
}