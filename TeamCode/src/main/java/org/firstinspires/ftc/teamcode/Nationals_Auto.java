package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "HolonomicOdometry")
public class Nationals_Auto extends LinearOpMode {

    private HolonomicOdometry odometry;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private MotorEx leftEncoder, rightEncoder, lateralEncoder;

    @Override
    public void runOpMode() {
        // Initialize encoders
        leftEncoder = new MotorEx(hardwareMap, "leftEncoder");
        rightEncoder = new MotorEx(hardwareMap, "rightEncoder");
        lateralEncoder = new MotorEx(hardwareMap, "lateralEncoder");

        // Odometry parameters
        final double TRACK_WIDTH = 0.205;
        final double LATERAL_DISTANCE = 0.13; // Offset of the lateral encoder
        final double TICKS_PER_REV = 2000;
        final double WHEEL_RADIUS = 0.016; // MIGHT NEED IN INCHES?
        final double DISTANCE_PER_TICK = (2 * Math.PI * WHEEL_RADIUS) / TICKS_PER_REV;

        //0.13 0.205

        // Create the odometry instance
        odometry = new HolonomicOdometry(
                () -> leftEncoder.getCurrentPosition() * DISTANCE_PER_TICK,
                () -> rightEncoder.getCurrentPosition() * DISTANCE_PER_TICK,
                () -> lateralEncoder.getCurrentPosition() * DISTANCE_PER_TICK,
                TRACK_WIDTH, LATERAL_DISTANCE
        );

        // Set initial position (optional)
        odometry.updatePose(new Pose2d(0, 0, new Rotation2d()));

        waitForStart();

        while (opModeIsActive()) {
            odometry.updatePose(); // Update the robot's position
            telemetry.addData("Pose", odometry.getPose());
            telemetry.update();

            moveDistance(2,0.5, 0.25);2
        }
    }

    public void moveToPose(Pose2d targetPose, double kP, double turnKP) {
        while (opModeIsActive() && !isAtTarget(targetPose)) {
            // Update odometry
            odometry.updatePose();
            Pose2d currentPose = odometry.getPose();

            // Calculate position errors
            double xError = targetPose.getX() - currentPose.getX();
            double yError = targetPose.getY() - currentPose.getY();
            double headingError = targetPose.getRotation().getDegrees() - currentPose.getRotation().getDegrees();

            // Use proportional control to calculate motor powers
            double forwardPower = yError * kP;
            double strafePower = xError * kP;
            double turnPower = headingError * turnKP;

            // Apply motor powers
            setMotorPowers(forwardPower, strafePower, turnPower);
        }
    }

    private boolean isAtTarget(Pose2d targetPose) {
        double positionTolerance = 1.0; // inches
        double angleTolerance = Math.toRadians(2); // degrees
        Pose2d currentPose = odometry.getPose();

        return Math.abs(targetPose.getX() - currentPose.getX()) < positionTolerance &&
                Math.abs(targetPose.getY() - currentPose.getY()) < positionTolerance &&
                Math.abs(targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians()) < angleTolerance;
    }

    private void setMotorPowers(double forward, double strafe, double turn) {
        double frontLeft = forward + strafe + turn;
        double frontRight = -forward + strafe - turn;
        double backLeft = -forward + strafe - turn;
        double backRight = forward + strafe - turn;
        double max;

        max = Math.max(Math.abs(frontLeft), Math.abs(frontRight));
        max = Math.max(max, Math.abs(backLeft));
        max = Math.max(max, Math.abs(backRight));

        frontLeftMotor.setPower(frontLeft);
        frontRightMotor.setPower(frontRight);
        backLeftMotor.setPower(backLeft);
        backRightMotor.setPower(backRight);

        if (max > 1.0) {
            frontLeft /= max;
            frontRight /= max;
            backRight /= max;
            backLeft /= max;
        }

    }

    public void turnToAngle(double targetAngle, double turnKP) {
        while (opModeIsActive() && !isAtTargetAngle(targetAngle)) {
            // Update odometry
            odometry.updatePose();
            double currentAngle = odometry.getPose().getRotation().getDegrees();
            double error = targetAngle - currentAngle;

            // Proportional control
            double turnPower = error * turnKP;

            // Apply turning power
            setMotorPowers(0, 0, turnPower);
        }
    }

    private boolean isAtTargetAngle(double targetAngle) {
        double angleTolerance = 2; // degrees
        double currentAngle = odometry.getPose().getRotation().getDegrees();
        return Math.abs(targetAngle - currentAngle) < angleTolerance;
    }

    public void moveDistance(double distanceMeters, double kP, double turnKP) {
        // Update odometry to get the current pose
        odometry.updatePose();
        Pose2d currentPose = odometry.getPose();

        // Calculate target pose
        double targetX = currentPose.getX() + distanceMeters * Math.cos(currentPose.getRotation().getRadians());
        double targetY = currentPose.getY() + distanceMeters * Math.sin(currentPose.getRotation().getRadians());

        // Move to the target pose
        moveToPose(new Pose2d(targetX, targetY, currentPose.getRotation()), kP, turnKP);
    }
//LEFT STRAFE POSITIVE

}
