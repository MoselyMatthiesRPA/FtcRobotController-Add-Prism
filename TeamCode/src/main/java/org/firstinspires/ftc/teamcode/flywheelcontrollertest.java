package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "flywheel controller test", group = "Competition")

public class flywheelcontrollertest extends OpMode {
    DualPidMotor flywheel;

    @Override
    public void init() {
        flywheel = new DualPidMotor (hardwareMap, "topflywheel", "bottomflywheel");
        flywheel.setVelocity(2000);
        flywheel.Update();
    }

    @Override
    public void loop() {

    }
}
