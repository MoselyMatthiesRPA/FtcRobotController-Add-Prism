package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

// This opmode is for tuning a PIDF controller(really just PF since that is all thats needed, I and D are set to 0) on a flywheel for the high velocity setting.
// The low velocity is just to simulate a slow-down so that we can test how our controller ramps up for tuning purposes.
// A PF should be tuned for one specific RPM, however you can mess with a scaler on your F to adjust for more distances(Divide F by the RPM and take that constant and multiply it by the target RPM when used)
// Keep in mind these velocities are in ticks per second, but you can convert to rpm by taking your desired RPM and solving
// (RPM*28)/60, with 28 being the encoder ticks per output rotation in this instance(bare motor). This does still use the rev PID
// though, which while it is good its not perfect and going full custom and only using encoders for measurement is better.
@TeleOp
public class PIDtuner extends OpMode {
    public DcMotorEx Flywheel;

    public double targetRPM = 3000;
    public double highVelocity = ((targetRPM * 28)/60);
    public double lowVelocity = 900;
    public double intakeTargetRPM = 500;

    double curTargetVelocity = highVelocity;

double F = 12.2;
    double P = 200;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.0001};
    int stepIndex = 1;

    @Override
    public void init(){
        Flywheel = hardwareMap.get(DcMotorEx.class, "backleft");
        Flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Flywheel.setDirection(DcMotor.Direction.REVERSE);



        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        Flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

    }
     @Override
    public void loop(){
        if (gamepad1.yWasPressed()){
            if (curTargetVelocity == highVelocity){
                curTargetVelocity = lowVelocity;
            } else{
                curTargetVelocity = highVelocity;
            }
        }

        if (gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()){
            F -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()){
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()){
            P += stepSizes[stepIndex];
        }

         if (gamepad1.dpadDownWasPressed()) {
             P -= stepSizes[stepIndex];
         }


         PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
         Flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

         Flywheel.setVelocity(curTargetVelocity);

         double curVelocity = (Flywheel.getVelocity());
         double error = curTargetVelocity - curVelocity;

         telemetry.addData("target velocity", curTargetVelocity);
         telemetry.addData("current velocity", "%.2f",curVelocity);
         telemetry.addData("error", "%.2f", error);
         telemetry.addData("tuning P", "%.4f (D pad U/D", P);
         telemetry.addData("tuning F", "%.4f (D pad L/R", F);
         telemetry.addData("Step size", "%.4f (B button)", stepSizes[stepIndex]);
         telemetry.addData("error", "%.2f", error);
     }
}
