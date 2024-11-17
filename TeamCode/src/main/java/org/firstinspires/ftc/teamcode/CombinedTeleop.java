package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Combined Teleop")
public class CombinedTeleop extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor leftSlideMotor;
    private DcMotor rightSlideMotor;
    private Servo extensionServo;
    private IMU imu;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        initHardware();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running (%d s)", runtime.seconds());
            updateDrive();
            updateSlides();
            updateExtension();
            telemetry.update();
        }
    }

    private void initHardware() {
        // DRIVE MOTORS
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        // the front right motor is reversed for some reason
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        final IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(params);

        // EXTENSION SERVO
        extensionServo = hardwareMap.get(Servo.class, "extensionServo");

        // SLIDE MOTORS
        leftSlideMotor = hardwareMap.get(DcMotor.class, "leftSlideMotor");
        rightSlideMotor = hardwareMap.get(DcMotor.class, "rightSlideMotor");
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // we use the slides for climbing as well, so we should brake on zero power
        leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // from https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    private static final double SLIDE_ENCODER_TICKS_PER_ROTATION = ((((1 + (46.0 / 17.0))) * (1 + (46.0 / 11.0))) * 28);
    // from https://www.gobilda.com/4-stage-viper-slide-kit-cable-driven-336mm-slides/
    private static final double SLIDE_SPOOL_ROTATIONS_TO_MAX_EXTENSION = 976.0 / 112.0;

    private void updateSlides() {
        // the a, b and x buttons on the controller are on top of each other in that order
        // so it kind of intuitively makes sense that a is no extension, b is 1/2 and x is full?
        // tbh i was simply not bothered to implement the code to detect separate d-pad presses

        if (gamepad1.a) {
            leftSlideMotor.setTargetPosition(0);
            rightSlideMotor.setTargetPosition(0);
        } else if (gamepad1.b) {
            leftSlideMotor.setTargetPosition((int) ((SLIDE_ENCODER_TICKS_PER_ROTATION * SLIDE_SPOOL_ROTATIONS_TO_MAX_EXTENSION) / 2.0));
            rightSlideMotor.setTargetPosition((int) ((SLIDE_ENCODER_TICKS_PER_ROTATION * SLIDE_SPOOL_ROTATIONS_TO_MAX_EXTENSION) / 2.0));
        } else if (gamepad1.x) {
            leftSlideMotor.setTargetPosition((int) (SLIDE_ENCODER_TICKS_PER_ROTATION * SLIDE_SPOOL_ROTATIONS_TO_MAX_EXTENSION));
            rightSlideMotor.setTargetPosition((int) (SLIDE_ENCODER_TICKS_PER_ROTATION * SLIDE_SPOOL_ROTATIONS_TO_MAX_EXTENSION));
        }

        // the target positions for the slides are the same for both motors, so we can just use one of them
        telemetry.addData("Slide Target Spool Rotations", leftSlideMotor.getTargetPosition() / SLIDE_ENCODER_TICKS_PER_ROTATION);
        telemetry.addData("Slide Target Extension %", leftSlideMotor.getTargetPosition() / (SLIDE_ENCODER_TICKS_PER_ROTATION * SLIDE_SPOOL_ROTATIONS_TO_MAX_EXTENSION));

        telemetry.addData("Left Slide Spool Rotations", leftSlideMotor.getCurrentPosition() / SLIDE_ENCODER_TICKS_PER_ROTATION);
        telemetry.addData("Left Slide Extension %", leftSlideMotor.getCurrentPosition() / (SLIDE_ENCODER_TICKS_PER_ROTATION * SLIDE_SPOOL_ROTATIONS_TO_MAX_EXTENSION));
        telemetry.addData("Right Slide Spool Rotations", rightSlideMotor.getCurrentPosition() / SLIDE_ENCODER_TICKS_PER_ROTATION);
        telemetry.addData("Right Slide Extension %", rightSlideMotor.getCurrentPosition() / (SLIDE_ENCODER_TICKS_PER_ROTATION * SLIDE_SPOOL_ROTATIONS_TO_MAX_EXTENSION));
    }


    private void updateExtension() {
        // for some reason setPosition() affects velocity, not position...
        // it also takes input from 0-1, where 0 is maximum left, 1 is maximum right, and 0.5 is no movement
        // it's weird, but it is what it is
        if (gamepad1.dpad_right) {
            extensionServo.setPosition(1);
        } else if (gamepad1.dpad_left) {
            extensionServo.setPosition(0);
        } else {
            extensionServo.setPosition(0.5);
        }

        telemetry.addData("Extension Servo Speed", extensionServo.getPosition());
    }

    private void updateDrive() {
        double inputY = -gamepad1.left_stick_y; // invert y input because controllers are weird
        double inputX = gamepad1.left_stick_x;
        double inputR = gamepad1.right_stick_x;
        driveFieldRelative(inputX, inputY, inputR);
    }

    private void driveFieldRelative(double x, double y, double r) {
        // https://en.wikipedia.org/wiki/Rotation_matrix
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double robotRelativeY = x * Math.cos(yaw) + y * Math.sin(yaw);
        double robotRelativeX = -x * Math.sin(yaw) + y * Math.cos(yaw);
        drive(robotRelativeX, robotRelativeY, r);
    }

    private void drive(double x, double y, double r) {
        double frontLeftPower = x - y - r;
        double frontRightPower = x + y + r;
        double backLeftPower = x + y - r;
        double backRightPower = x - y + r;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Send powers to the wheels.
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);

        telemetry.addData("FL Drive Power", frontLeftPower);
        telemetry.addData("FR Drive Power", frontRightPower);
        telemetry.addData("BL Drive Power", backLeftPower);
        telemetry.addData("BR Drive Power", backRightPower);
    }
}
