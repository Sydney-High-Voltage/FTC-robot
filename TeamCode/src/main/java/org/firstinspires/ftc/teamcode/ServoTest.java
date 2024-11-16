package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test")
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        final Servo servo = hardwareMap.servo.get("servo");

        waitForStart();
        while (opModeIsActive()) {
            servo.setPosition((gamepad1.right_stick_x + 1) / 2);
        }
    }
}
