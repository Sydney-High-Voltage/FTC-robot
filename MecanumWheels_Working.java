package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name = "drive")
public class MecanumWheels_Working extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private IMU imu = null;
    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // allow driver to reset yaw
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double inputY = -gamepad1.left_stick_y;
            double inputX = gamepad1.left_stick_x;
            double rot = gamepad1.right_stick_x;

            // convert robot relative velocity to field relative velocity via 2d vector rotation
            // https://en.wikipedia.org/wiki/Rotation_matrix
            double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double y = inputX * Math.cos(yaw) + inputY * Math.sin(yaw);
            double x = -inputX * Math.sin(yaw) + inputY * Math.cos(yaw);

            double frontLeftPower = y + x + rot;
            double frontRightPower = y - x - rot;
            double backLeftPower = y - x + rot;
            double backRightPower = y + x - rot;

            // make sure we're not setting motor powers >1.0
            double max;
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));
            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Yaw Angle (deg)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("FL Power", frontLeftPower);
            telemetry.addData("FR Power", frontRightPower);
            telemetry.addData("BL Power", backLeftPower);
            telemetry.addData("BR Power", backRightPower);
            telemetry.update();
        }
    }
}

