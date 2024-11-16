package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class AUTO extends LinearOpMode {
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor armTilt;
    DcMotor armExtend;

    BNO055IMU imu;

    @Override
    public void runOpMode() {
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        double fl, fr, bl, br;
        fl = 1;
        fr = 1;
        bl = -1;
        br = -1;
        int t = 0;
        while (opModeIsActive()) {
            frontLeftDrive.setPower(fl);
            frontRightDrive.setPower(fr);
            backLeftDrive.setPower(bl);
            backRightDrive.setPower(br);
            t++;
            if (t >= 40) {
                fl = -1;
                fr = 1;
                bl = -1;
                br = 1;
            }
            if (t >= 100) {
                fl = 1;
                fr = 1;
                bl = -1;
                br = -1;
            }
            if (t >= 120) {
                fl = 1;
                fr = -1;
                bl = 1;
                br = -1;
            }
            if (t >= 160) {
                fl = -1;
                fr = -1;
                bl = 1;
                br = 1;
            }
            if (t >= 180) {
                fl = 1;
                fr = -1;
                bl = 1;
                br = -1;
            }
            if (t >= 200) {
                fl = 1;
                fr = 1;
                bl = -1;
                br = -1;
            }
            if (t >= 240) {
                fl = -1;
                fr = -1;
                bl = 1;
                br = 1;
            }
            if (t >= 600) {
                fl = 1;
                fr = -1;
                bl = 1;
                br = -1;
            }
            if (t >= 1000) {
                fl = 0;
                fr = 0;
                bl = 0;
                br = 0;
            }


        }

    }
}
