package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


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
            // allow driver to reset yaw
            if (gamepad1.options) {
                imu.resetYaw();
            }

            // get joystick inputs
            double inputY = -gamepad1.left_stick_y;
            double inputX = gamepad1.left_stick_x;
            double rot = gamepad1.right_stick_x;

            // this converts the input velocity, which is robot-relative, to field relative velocity via 2d vector rotation (https://en.wikipedia.org/wiki/Rotation_matrix)
            // this allows the driver to control the robot using motions relative to the field rather than the robot.
            // i assume this is what you were trying to do in the other files, but... failed.
            // also, ftclib has lots of classes that do these calculations for you. if you want to use those, find docs for the MecanumDriveKinematics and ChassisSpeeds classes, among others.
            // alternatively, feel free to ping me on the discord if you get stuck.
            // - neel
            double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double y = inputX * Math.cos(yaw) + inputY * Math.sin(yaw);
            double x = -inputX * Math.sin(yaw) + inputY * Math.cos(yaw);

            // fyi the rest of this code is mainly taken from the example mecanum drive code that ftc provides.
            // - neel
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

