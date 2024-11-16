package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Represents an omni-directional drive system with four motors on the four corners of the robot.
 */
public class OmniDrive {
    private final DcMotor frontLeftMotor;
    private final DcMotor frontRightMotor;
    private final DcMotor backLeftMotor;
    private final DcMotor backRightMotor;

    /**
     * Constructs a new {@link OmniDrive}.
     *
     * @param frontLeftMotor  The front left motor.
     * @param frontRightMotor The front right motor.
     * @param backLeftMotor   The back left motor.
     * @param backRightMotor  The back right motor.
     */
    public OmniDrive(DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor) {
        this.frontLeftMotor = frontLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.backLeftMotor = backLeftMotor;
        this.backRightMotor = backRightMotor;
    }

    /**
     * Drives the robot with the provided robot-relative velocities.
     *
     * @param x The robot-relative x velocity.
     * @param y The robot-relative y velocity.
     * @param r The robot-relative rotational velocity.
     */
    public void drive(double x, double y, double r) {
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
        setPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    /**
     * Sets the powers to the 4 motors.
     *
     * @param frontLeftPower  Power to the front left motor
     * @param frontRightPower Power to the front right motor
     * @param backLeftPower   Power to the back left motor
     * @param backRightPower  Power to the back right motor
     */
    public void setPowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    /**
     * Stops all motors
     */
    public void stop() {
        setPowers(0, 0, 0, 0);
    }
}
