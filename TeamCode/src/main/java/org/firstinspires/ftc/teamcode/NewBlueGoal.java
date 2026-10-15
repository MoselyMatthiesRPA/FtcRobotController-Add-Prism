package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.lynx.LynxModule;
import java.util.List;

@Configurable
@TeleOp(name = "BlueOp", group = "Competition")
public class NewBlueGoal extends OpMode {
    public double targetRPM = 2600;
        public double highVelocity = ((targetRPM * 28)/60);
    public double lowVelocity = 900;

    double curTargetVelocity = highVelocity;

    double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001, 0.000001, 0.0000001};
    int stepIndex = 1;
    DcMotorEx flywheel;

    // Live-tunable PID gains
    double F = 0;
    double P = 0;
//    public double kI = 0.0;

    private double integralSum = 0;
    private double lastError = 0;
    private final ElapsedTime pidTimer = new ElapsedTime();

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "1");
//        flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        flywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

    }

    @Override
    public void start() {
        pidTimer.reset();
    }

    @Override
    public void loop() {
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

        if (gamepad1.dpadDownWasPressed()){
            F -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()){
            F += stepSizes[stepIndex];
        }

//
        if (gamepad1.aWasPressed()){
            P += stepSizes[stepIndex];
        }

         if (gamepad1.xWasPressed()) {
             P -= stepSizes[stepIndex];
         }
            double targetTicksPerSec = ((3500*28)/60);
            double currentTicksPerSec = flywheel.getVelocity();

            double error = targetTicksPerSec - currentTicksPerSec;

            double power = (F * targetTicksPerSec) + (P * error);
            power = Math.max(-1.0, Math.min(1.0, power)); // clamp to valid motor power range

            flywheel.setPower(power);
            telemetry.addData("PID F", F);
            telemetry.addData("PID P", P);
            telemetry.addData("Step size", "%.4f (B button)", stepSizes[stepIndex]);
            telemetry.addData("error", "%.2f", error);
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Current RPM", currentTicksPerSec * 60 / 28);
            telemetry.addData("Power", power);
            telemetry.update();
    }
}