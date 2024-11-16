package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Slide Control")
public class SlideControl extends LinearOpMode {
    // from https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    private static final double ENCODER_TICKS_PER_ROTATION = ((((1 + (46.0 / 17.0))) * (1 + (46.0 / 11.0))) * 28);
    // from https://www.gobilda.com/4-stage-viper-slide-kit-cable-driven-336mm-slides/
    private static final double SPOOL_ROTATIONS_TO_MAX_EXTENSION = 976.0 / 112.0;

    private DcMotorEx motor;
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPositionPIDFCoefficients(0.1);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                motor.setTargetPosition(rotationsToTicks(0));
            } else if (gamepad1.b) {
                motor.setTargetPosition(rotationsToTicks(SPOOL_ROTATIONS_TO_MAX_EXTENSION / 2));
            } else if (gamepad1.x) {
                motor.setTargetPosition(rotationsToTicks(SPOOL_ROTATIONS_TO_MAX_EXTENSION - 0.2));
            }
            telemetry.addData("Status", "Running (%d s)", runtime.seconds());
        }
    }

    private int rotationsToTicks(double rotations) {
        return (int) (rotations * ENCODER_TICKS_PER_ROTATION);

    }

    private double getEncoderRotations() {
        return motor.getCurrentPosition() / ENCODER_TICKS_PER_ROTATION;
    }
}
