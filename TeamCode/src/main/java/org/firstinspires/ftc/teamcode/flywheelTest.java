package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Flywheeltest", group = "Competition")

public class flywheelTest extends OpMode {
    public DcMotor intake;
    @Override
    public void init(){
        intake = hardwareMap.get(DcMotor.class, "turret");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setPower(1);

    }

    @Override
    public void loop() {

    }
}
