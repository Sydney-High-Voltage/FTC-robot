package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

// Main TeleOp
@TeleOp(name = "Main Dec 7 nationals")
public class _Main extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotorEx leftSlideMotor;
    private DcMotorEx rightSlideMotor;
    private boolean c;
    private CRServo extensionServo;
    private CRServo rotationServo;
    private IMU imu;
    private int intaktrigger = 0;
    private boolean intakemode = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        initHardware();

        waitForStart();

        runtime.reset();
        long lastLoopTime = System.nanoTime();

        while (opModeIsActive()) {
            //telemetry.addData("Status", "Running (%d s)", runtime.seconds());
            updateDrive();
            updateSlides();
            updateExtension();
            long currentTime = System.nanoTime();
            long loopcycle = currentTime - lastLoopTime;
            lastLoopTime = currentTime;
            double lts = loopcycle / 1000_000_000.0;
            lts = 1 / lts;
            telemetry.addData("loop cycle in hz", lts);

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
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        // IMU
//        imu = hardwareMap.get(IMU.class, "imu");
//        //rightSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        final IMU.Parameters params = new IMU.Parameters(
//                new RevHubOrientationOnRobot(
//                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
//                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
//        imu.initialize(params);

        // EXTENSION SERVO
        extensionServo = hardwareMap.get(CRServo.class, "extensionServo");
        rotationServo = hardwareMap.get(CRServo.class, "rotationServo");
        // SLIDE MOTORS
        leftSlideMotor = hardwareMap.get(DcMotorEx.class, "leftSlideMotor");
        rightSlideMotor = hardwareMap.get(DcMotorEx.class, "rightSlideMotor");
        rightSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // we use the slides for climbing as well, so we should brake on zero power
        leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlideMotor.setPositionPIDFCoefficients(2.5);
        rightSlideMotor.setPositionPIDFCoefficients(2.5);

        // target position must be set prior to setting mode to RUN_TO_POSITION
        leftSlideMotor.setTargetPosition(0);
        rightSlideMotor.setTargetPosition(0);
        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlideMotor.setPower(1);
        rightSlideMotor.setPower(1);
    }

    // from https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    private static final double SLIDE_ENCODER_TICKS_PER_ROTATION = ((((1 + (46.0 / 17.0))) * (1 + (46.0 / 11.0))) * 28);
    // from https://www.gobilda.com/4-stage-viper-slide-kit-cable-driven-336mm-slides/
    private static final double SLIDE_SPOOL_ROTATIONS_TO_MAX_EXTENSION = 976.0 / 120.0 * 1.055;

    private void updateSlides() {
        // the a, b and x buttons on the controller are on top of each other in that order
        // so it kind of intuitively makes sense that a is no extension, b is 1/2 and x is full?
        // tbh i was simply not bothered to implement the code to detect separate d-pad presses

        if (gamepad1.a || gamepad2.a) {
            leftSlideMotor.setTargetPosition(0);
            rightSlideMotor.setTargetPosition(0);
        } else if (gamepad1.b || gamepad2.b) {
            leftSlideMotor.setTargetPosition((int) ((SLIDE_ENCODER_TICKS_PER_ROTATION * SLIDE_SPOOL_ROTATIONS_TO_MAX_EXTENSION) / 2.0));
            rightSlideMotor.setTargetPosition((int) ((SLIDE_ENCODER_TICKS_PER_ROTATION * SLIDE_SPOOL_ROTATIONS_TO_MAX_EXTENSION) / 2.0));
        } else if (gamepad1.x || gamepad2.x) {
            leftSlideMotor.setTargetPosition((int) (SLIDE_ENCODER_TICKS_PER_ROTATION * SLIDE_SPOOL_ROTATIONS_TO_MAX_EXTENSION));
            rightSlideMotor.setTargetPosition((int) (SLIDE_ENCODER_TICKS_PER_ROTATION * SLIDE_SPOOL_ROTATIONS_TO_MAX_EXTENSION));
        }


        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlideMotor.setPower(1);
        rightSlideMotor.setPower(1);
        leftSlideMotor.setVelocity(1e9);
        rightSlideMotor.setVelocity(1e9);
        c = gamepad1.a || gamepad2.a || gamepad1.b || gamepad2.b || gamepad1.x || gamepad2.x;
        if (!c && Math.abs(leftSlideMotor.getCurrentPosition() - leftSlideMotor.getTargetPosition()) < 15) {
            leftSlideMotor.setPower(0);
            rightSlideMotor.setPower(0);
            leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        // the target positions for the slides are the same for both motors, so we can just use one of them
        telemetry.addData("Slide Target Spool Rotations", leftSlideMotor.getTargetPosition() / SLIDE_ENCODER_TICKS_PER_ROTATION);
        telemetry.addData("Slide Target Extension %", leftSlideMotor.getTargetPosition() / (SLIDE_ENCODER_TICKS_PER_ROTATION * SLIDE_SPOOL_ROTATIONS_TO_MAX_EXTENSION));

        telemetry.addData("Left Slide Spool Rotations", leftSlideMotor.getCurrentPosition() / SLIDE_ENCODER_TICKS_PER_ROTATION);
        telemetry.addData("Left Slide Extension %", leftSlideMotor.getCurrentPosition() / (SLIDE_ENCODER_TICKS_PER_ROTATION * SLIDE_SPOOL_ROTATIONS_TO_MAX_EXTENSION));
        telemetry.addData("Right Slide Spool Rotations", rightSlideMotor.getCurrentPosition() / SLIDE_ENCODER_TICKS_PER_ROTATION);
        telemetry.addData("Right Slide Extension %", rightSlideMotor.getCurrentPosition() / (SLIDE_ENCODER_TICKS_PER_ROTATION * SLIDE_SPOOL_ROTATIONS_TO_MAX_EXTENSION));

        telemetry.addData("slide Target", leftSlideMotor.getTargetPosition());
        telemetry.addData("slide current position", leftSlideMotor.getCurrentPosition());
    }


    private void updateExtension() {
        // for some reason setPosition() affects velocity, not position...
        // it also takes input from 0-1, where 0 is maximum left, 1 is maximum right, and 0.5 is no movement
        // it's weird, but it is what it is
        if (gamepad1.right_bumper || gamepad2.right_bumper) {
            if (intaktrigger == 0) {
                intakemode = !intakemode;
                intaktrigger = 1;
            }
            extensionServo.setPower(-1);
        } else if (gamepad1.left_trigger > 0 || gamepad2.left_trigger > 0) {
            extensionServo.setPower(0.4);
        } else {
            if (!intakemode) extensionServo.setPower(0);
            intaktrigger = 0;
        }
        if (intakemode)
            extensionServo.setPower(-1);

        if (gamepad1.right_trigger > 0 || gamepad2.right_trigger > 0) {
            rotationServo.setPower(1);
        } else if (gamepad1.left_bumper || gamepad2.left_bumper) {
            rotationServo.setPower(-1);
        } else {
            rotationServo.setPower(0);
        }

    }

    private void updateDrive() {
        double inputY = -gamepad1.left_stick_y; // invert y input because controllers are weird
        double inputX = gamepad1.left_stick_x;
        double inputR = gamepad1.right_stick_x * 0.5;

        // field-rel is broken because of the imu only reporting yaw between -90 and 90 degrees
        // TODO: fix
        //driveFieldRelative(inputX, inputY, inputR);
        telemetry.addData("Input X", gamepad1.left_stick_x);
        drive(-inputX, -inputY, inputR);
    }

//    private void driveFieldRelative(double x, double y, double r) {
//        // https://en.wikipedia.org/wiki/Rotation_matrix
//        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//        double robotRelativeY = x * Math.cos(yaw) + y * Math.sin(yaw);
//        double robotRelativeX = -x * Math.sin(yaw) + y * Math.cos(yaw);
//        drive(x, y, r);
//        //drive(robotRelativeX, robotRelativeY, r);
//    }

    private void drive(double x, double y, double r) {

        double frontLeftPower = x + y + r;
        double frontRightPower = -x + y - r;
        double backLeftPower = -x + y + r;
        double backRightPower = x + y - r;

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
        telemetry.addData("Left Slide Current", leftSlideMotor.getCurrent(CurrentUnit.AMPS));
    }
}
