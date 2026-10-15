package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Configurable
public class DualPidMotor {
    private DcMotorEx Flywheel;
    double targetRPM;
    public static double P = 50;
    double kf = 0.00380387803878049; // Constant used to create F by multiplying kf by targetRPM, allowing for scalability with different RPMs.
    public double F;

    public DualPidMotor(HardwareMap hardwareMap, String FlywheelMotorName) {
        Flywheel = hardwareMap.get(DcMotorEx.class, FlywheelMotorName);
        Flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Flywheel.setDirection(DcMotorSimple.Direction.REVERSE);


        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        Flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }
    public void setVelocity(double targetRPM){
        this.targetRPM = targetRPM;
    }

    public void Update(){
        // Takes the Average of both motors' speeds in ticks per second and converts to RPM
     double targetVelocity = (targetRPM*28)/60;
        F = kf * targetRPM;
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0, F);
        Flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        Flywheel.setVelocity(targetVelocity);
    }
    public double getCurrentRPM() {
        double avgTicksPerSecond =
                Flywheel.getVelocity();

        return (avgTicksPerSecond * 60.0) / 28;
    }
}
