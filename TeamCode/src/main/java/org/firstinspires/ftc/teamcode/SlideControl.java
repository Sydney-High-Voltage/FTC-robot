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

    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftMotor = hardwareMap.get(DcMotorEx.class, "leftSlideMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightSlideMotor");
        leftMotor.setTargetPosition(0);
        rightMotor.setTargetPosition(0);
        leftMotor.setPositionPIDFCoefficients(1);
        rightMotor.setPositionPIDFCoefficients(1);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setPower(1);
        rightMotor.setPower(1);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                leftMotor.setTargetPosition(0);
                rightMotor.setTargetPosition(0);
            } else if (gamepad1.b) {
                leftMotor.setTargetPosition(rotationsToTicks(SPOOL_ROTATIONS_TO_MAX_EXTENSION * 0.5));
                rightMotor.setTargetPosition(rotationsToTicks(SPOOL_ROTATIONS_TO_MAX_EXTENSION / 2));
            } else if (gamepad1.x) {
                leftMotor.setTargetPosition(rotationsToTicks(SPOOL_ROTATIONS_TO_MAX_EXTENSION * 0.8));
                rightMotor.setTargetPosition(rotationsToTicks(SPOOL_ROTATIONS_TO_MAX_EXTENSION));
            }

            telemetry.addData("Status", runtime.seconds());
            telemetry.addData("Slide Target Distance Ticks", leftMotor.getTargetPosition());
            telemetry.addData("Slide Busy", leftMotor.isBusy());
            telemetry.addData("Slide Target Rotations", leftMotor.getTargetPosition() / ENCODER_TICKS_PER_ROTATION);
            telemetry.addData("Slide Target Extension %", leftMotor.getTargetPosition() / (ENCODER_TICKS_PER_ROTATION * SPOOL_ROTATIONS_TO_MAX_EXTENSION));
            telemetry.addData("Left Slide Spool Rotations", leftMotor.getCurrentPosition() / ENCODER_TICKS_PER_ROTATION);
            //telemetry.addData("Right Slide Spool Position", rightMotor.getCurrentPosition() / ENCODER_TICKS_PER_ROTATION);
            telemetry.addData("Left Slide Extension %", leftMotor.getCurrentPosition() / (ENCODER_TICKS_PER_ROTATION * SPOOL_ROTATIONS_TO_MAX_EXTENSION));
            //telemetry.addData("Right Slide Extension %", rightMotor.getCurrentPosition() / (ENCODER_TICKS_PER_ROTATION * SPOOL_ROTATIONS_TO_MAX_EXTENSION));
            telemetry.update();
        }
    }

    private int rotationsToTicks(double rotations) {
        return (int) (rotations * ENCODER_TICKS_PER_ROTATION);
    }
}
