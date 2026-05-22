package org.firstinspires.ftc.teamcode.lawrence.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lawrence.constants.HardwareNames;

@Configurable
public class Turret {

    private final DcMotorEx motor;

    // ── Tuning (static so Configurable dashboard can adjust at runtime) ───────
    public static double kP = 0.03;
    public static double kD = 0.0008;
    // Directional feedforward — provides minimum torque to overcome static friction
    public static double kF = 0.043;

    public static double MAX_ANGLE    =  140.0;
    public static double MIN_ANGLE    = -160.0;
    public static double ERROR_DEADBAND = 0.18; // degrees

    // ── Encoder geometry ──────────────────────────────────────────────────────
    // GoBILDA 5203 motor: 145.1 ticks/rev at the motor shaft
    // External stage: 110:20 reduction
    // Combined: 145.1 × (110/20) = ~797.05 ticks per turret revolution
    private static final double TICKS_PER_TURRET_REV = 145.1 * (110.0 / 20.0);

    // ── State ─────────────────────────────────────────────────────────────────
    private double targetAngleDeg = 0;
    private double currentAngleDeg = 0;
    private double lastAngle = 0;
    public double output = 0;

    public Turret(HardwareMap hardwareMap, boolean resetEncoder) {
        motor = hardwareMap.get(DcMotorEx.class, HardwareNames.TURRET);
        if (resetEncoder) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTargetAngle(double degrees) {
        targetAngleDeg = degrees;
    }

    public double getTargetAngle() {
        return targetAngleDeg;
    }

    public double getCurrentAngle() {
        return currentAngleDeg;
    }

    public double getOutput() {
        return output;
    }

    public void update(double dt) {
        currentAngleDeg = ticksToDegrees(motor.getCurrentPosition());

        // Pick the equivalent angle (target ± 360°) that is within limits and
        // closest to current position — avoids spinning the long way around.
        double minChange = Double.MAX_VALUE;
        double bestError = 0;
        for (int k = -1; k <= 1; k++) {
            double candidate = targetAngleDeg + 360.0 * k;
            if (candidate >= MIN_ANGLE && candidate <= MAX_ANGLE) {
                double err = candidate - currentAngleDeg;
                if (Math.abs(err) < minChange) {
                    minChange = Math.abs(err);
                    bestError = err;
                }
            }
        }
        // If no candidate is in range, clamp to the nearest limit
        if (minChange == Double.MAX_VALUE) {
            bestError = (targetAngleDeg > MAX_ANGLE ? MAX_ANGLE : MIN_ANGLE) - currentAngleDeg;
        }

        // PD + directional feedforward (derivative skipped on first call when dt=0)
        double derivative = 0;
        if (dt > 0) {
            double velocity = (currentAngleDeg - lastAngle) / dt;
            derivative = -velocity;
        }
        lastAngle = currentAngleDeg;

        if (Math.abs(bestError) > ERROR_DEADBAND) {
            output = kP * bestError + kD * derivative + Math.signum(bestError) * kF;
        } else {
            output = 0;
        }

        motor.setPower(output);
    }

    private double ticksToDegrees(double ticks) {
        return (ticks / TICKS_PER_TURRET_REV) * 360.0;
    }
}
