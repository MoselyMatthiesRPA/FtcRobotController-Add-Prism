package org.firstinspires.ftc.teamcode.lawrence.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.lawrence.constants.HardwareNames;

@Configurable
public class Flywheel {

    private final DcMotorEx top;
    private final DcMotorEx bottom;

    public static double P  = 120.0;
    public static double kf = 0.00380387803878049;

    private double targetRPM = 0;
    private final PIDFCoefficients pidf = new PIDFCoefficients(P, 0, 0, 0);

    public Flywheel(HardwareMap hardwareMap) {
        top    = hardwareMap.get(DcMotorEx.class, HardwareNames.TOP_FLYWHEEL);
        bottom = hardwareMap.get(DcMotorEx.class, HardwareNames.BOTTOM_FLYWHEEL);

        top.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        top.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        bottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottom.setDirection(DcMotor.Direction.REVERSE);
        bottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        top.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        bottom.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    public void setTargetRPM(double rpm) {
        this.targetRPM = rpm;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public void update() {
        double targetVelocity = (targetRPM * 28.0) / 60.0;
        pidf.p = P;
        pidf.f = kf * targetRPM;
        top.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        bottom.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        top.setVelocity(targetVelocity);
        bottom.setVelocity(targetVelocity);
    }

    public double getCurrentRPM() {
        double avgTicksPerSec = (top.getVelocity() + bottom.getVelocity()) / 2.0;
        return (avgTicksPerSec * 60.0) / 28.0;
    }
}
