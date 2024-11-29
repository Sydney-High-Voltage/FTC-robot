package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto - Push Blocks")
public class _AutoPush extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotorEx leftSlideMotor;
    private DcMotorEx rightSlideMotor;
    private CRServo extensionServo;
    private CRServo rotationServo;
    private IMU imu;
    //measurements recorded (not in code rn but just in case anymore )800s -> 560cm
    public void runOpMode() {
        waitForStart();
        init_hardware();
        // left from start
        mv(0, 0.5, 0, 100);
        mv(-0.5, 0, 0, 800);
        //up
        mv(0, .4, 0, 2000);
        //left
        mv(-0.5, 0, 0, 400);
        // push it to area
        mv(0, -.4, 0, 1500);
        mv(0,0.5,0, 100);
        mv(0.5, 0, 0, 700);
        mv(0, -.5, 0.1, 300);
        mv(-0.5, 0, 0, 1300);

        //second block
        mv(0.5, 0, 0, 700);
        mv(0, 0.5, 0, 1200);
        mv(-0.5, 0, 0, 500);

        mv(0, -.4, 0, 1500);
        mv(0,0.5,0, 100);
        mv(0.5, 0, 0, 700);
        mv(0, -.5, 0, 300);
        mv(-0.5, 0, 0, 1300);


        mv(0, 0.5, 0, 1200);
        mv(-0.5, 0, 0, 500);

        mv(0, -.4, 0, 1500);
        mv(0,0.5,0, 100);
        mv(0.5, 0, 0, 700);
        mv(0, -.5, 0, 300);
        mv(-0.5, 0, 0, 1300);




    }


    public void init_hardware() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        // the front right motor is reversed for some reason
        //frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        //rightSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        final IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(params);

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

        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlideMotor.setPower(1);
        rightSlideMotor.setPower(1);
    }

    private void mv(double x, double y, double r, long milisec) {
        drive(r, -y, x);
        sleep(milisec);
        drive(0, 0, 0);
    }

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
    }
}
