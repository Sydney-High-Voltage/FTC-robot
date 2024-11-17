package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// NOTE:
// x refers to forward-backward movement
// y refers to left-right movement

@TeleOp(name = "field-relative drive")
public class FieldRelativeDrive extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private IMU imu;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        // btw, yall had reversed a motor that shouldn't have been reversed.
        // for future reference, if the robot isn't moving correctly, it's usually a good idea to prop the robot up so that the wheels don't touch the floor so you can inspect if wheels are turning the right way or not. that's how i discovered this.
        // - neel
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        final IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(params);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Yaw Angle (deg)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

            // allow driver to reset yaw
            if (gamepad1.options) {
                imu.resetYaw();
            }

            // get joystick inputs
            double inputX = -gamepad1.left_stick_y;
            double inputY = gamepad1.left_stick_x;
            double rot = gamepad1.right_stick_x;
            driveFieldRelative(inputX, inputY, rot);

            telemetry.update();
        }
    }

    private void driveFieldRelative(double x, double y, double r) {
        // https://en.wikipedia.org/wiki/Rotation_matrix
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double robotRelativeX = x * Math.cos(yaw) + y * Math.sin(yaw);
        double robotRelativeY = -x * Math.sin(yaw) + y * Math.cos(yaw);
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

