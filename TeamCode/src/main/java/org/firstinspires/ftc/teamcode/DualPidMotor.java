package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Configurable
public class DualPidMotor {
    private DcMotorEx topFlywheel, bottomFlywheel;
    double targetRPM;
    public static double P = 80;
    double kf = 0.00542307692307692; // Constant used to create F by multiplying kf by targetRPM, allowing for scalability with different RPMs.
    double F;

    public DualPidMotor(HardwareMap hardwareMap, String TopFlywheelMotorName, String BottomFlywheelMotorName) {
        topFlywheel = hardwareMap.get(DcMotorEx.class, TopFlywheelMotorName);
        bottomFlywheel = hardwareMap.get(DcMotorEx.class, BottomFlywheelMotorName);
        topFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomFlywheel.setDirection(DcMotor.Direction.REVERSE);
        topFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        topFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        bottomFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

    }
    public void setVelocity(double targetRPM){
        this.targetRPM = targetRPM;
    }

    public void Update(){
        // Takes the Average of both motors' speeds in ticks per second and converts to RPM
     double targetVelocity = (targetRPM*28)/60;
        F = kf * targetRPM;
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0, F);
        topFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        bottomFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        topFlywheel.setVelocity(targetVelocity);
        bottomFlywheel.setVelocity(targetVelocity);
    }
    public double getCurrentRPM() {
        double avgTicksPerSecond =
                (topFlywheel.getVelocity() + bottomFlywheel.getVelocity()) / 2.0;

        return (avgTicksPerSecond * 60.0) / 28;
    }
}
