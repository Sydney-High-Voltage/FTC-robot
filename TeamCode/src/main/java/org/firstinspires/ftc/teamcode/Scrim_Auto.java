package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;



public class Scrim_Auto extends LinearOpMode {
    DcMotor backLeftDrive = null;
    DcMotor backRightDrive = null;
    DcMotor frontLeftDrive = null;
    DcMotor frontRightDrive = null;
    BNO055IMU imu;

    @Override
    public void runOpMode () {
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        waitForStart();
        moveForward();
        sleep(2500);
        stopDriving();
        turnLeft(1);
        sleep(2500);
        stopDriving();
        moveForward();
        sleep(3400);
        stopDriving();
        backLeftDrive.setPower(1);
        frontLeftDrive.setPower(-1);
        sleep(5000);
        stopDriving();
        moveForward();
        sleep(1000);
        stopDriving();
        backRightDrive.setPower(-1);
        frontRightDrive.setPower(1);
        sleep(8000);
        backLeftDrive.setPower(0.5);
        frontLeftDrive.setPower(-0.5);
        backRightDrive.setPower(0.5);
        frontRightDrive.setPower(-0.5);
        sleep(1000);
        backLeftDrive.setPower(1);
        frontLeftDrive.setPower(1);
        backRightDrive.setPower(-1);
        frontRightDrive.setPower(-1);
        sleep(6000);
        stopDriving();
    }

    private void moveForward() {
        backLeftDrive.setPower(-0.5);
        frontLeftDrive.setPower(-0.5);
        backRightDrive.setPower(0.5);
        backLeftDrive.setPower(0.5);
    }

    private void stopDriving() {
        backLeftDrive.setPower(0);
        frontLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        backLeftDrive.setPower(0);
    }

    private void turnLeft(double power) {
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
    }
}