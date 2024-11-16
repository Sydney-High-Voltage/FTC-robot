package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "Auto Concept")
public class AutoConcept extends LinearOpMode {
    private static double ODOMETRY_TICKS_PER_REVOLUTION = 2000;
    private static double ODOMETRY_POD_DIAMETER = 0.032;
    private static double ODOMETRY_POD_DISTANCE_PER_TICK = Math.PI * ODOMETRY_POD_DIAMETER / ODOMETRY_TICKS_PER_REVOLUTION;

    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    private DcMotor leftOdometryPod;
    private DcMotor rightOdometryPod;

    private IMU imu;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        leftOdometryPod = hardwareMap.get(DcMotor.class, "leftOdometryPod");
        rightOdometryPod = hardwareMap.get(DcMotor.class, "rightOdometryPod");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        final IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(params);

        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        aprilTagProcessor.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .addProcessor(aprilTagProcessor)
                .build();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double inputY = -gamepad1.left_stick_y;
            double inputX = gamepad1.left_stick_x;
            double inputR = gamepad1.right_stick_x;
            driveFieldRelative(inputX, inputY, inputR);
        }
    }

    private void driveDistance(double targetDistance) {
        double initialDistance = getDistance();
        double currentDistance = initialDistance;

        while (Math.abs(currentDistance - initialDistance) < Math.abs(targetDistance)) {
            double y = targetDistance - initialDistance > 0 ? 1 : -1;
            drive(0, y, 0);
            currentDistance = getDistance();
        }
        drive(0, 0, 0);
    }

    private void turnAngle(double targetAngle) {
        double initialAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double currentAngle = initialAngle;
        while (Math.abs(initialAngle - currentAngle) < Math.abs(targetAngle)) {
            double r = targetAngle - initialAngle > 0 ? 1 : -1;
            drive(0, 0, r);
            currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }
        drive(0, 0, 0);
    }

    private double getDistance() {
        double leftDistance = leftOdometryPod.getCurrentPosition() * ODOMETRY_POD_DISTANCE_PER_TICK;
        double rightDistance = rightOdometryPod.getCurrentPosition() * ODOMETRY_POD_DISTANCE_PER_TICK;
        return (leftDistance + rightDistance) / 2;
    }

    private void driveFieldRelative(double x, double y, double r) {
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

        telemetry.addData("FL Power", frontLeftPower);
        telemetry.addData("FR Power", frontRightPower);
        telemetry.addData("BL Power", backLeftPower);
        telemetry.addData("BR Power", backRightPower);
    }
}
