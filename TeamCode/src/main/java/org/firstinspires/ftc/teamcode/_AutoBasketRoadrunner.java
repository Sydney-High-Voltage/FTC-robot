package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "", group = "Autonomous")
public class _AutoBasketRoadrunner extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftSlideMotor;
    private DcMotorEx rightSlideMotor;
    private CRServo extensionServo;
    private CRServo rotationServo;
    private IMU imu;
    private static final double SLIDE_ENCODER_TICKS_PER_ROTATION = ((((1 + (46.0 / 17.0))) * (1 + (46.0 / 11.0))) * 28);
    // from https://www.gobilda.com/4-stage-viper-slide-kit-cable-driven-336mm-slides/
    private static final double SLIDE_SPOOL_ROTATIONS_TO_MAX_EXTENSION = 976.0 / 120.0 * 1.055;
    @Override
    public void runOpMode() {
        initHardware();
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // make a Lift instance
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3);
    }
    private void initHardware() {
        // DRIVE MOTORS
        /*
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        // the front right motor is reversed for some reason
        //frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        */

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        //rightSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        final IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(params);
        /*
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        */

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
    private void extend(){
        extensionServo.setPower(1);
        sleep(1000);
        extensionServo.setPower(0);
    }
    private void shorten(){
        extensionServo.setPower(-1);
        sleep(1000);
        extensionServo.setPower(0);
    }
    private void intake(){
        rotationServo.setPower(0.6);

    }
    private void outtake(){
        rotationServo.setPower(-.6);
        sleep(500);
        rotationServo.setPower(0);
    }
    private void slideTarget(int mode){

        if (mode == 0){
            leftSlideMotor.setTargetPosition(0);
            rightSlideMotor.setTargetPosition(0);}
        else if (mode ==1){
            leftSlideMotor.setTargetPosition((int) ((SLIDE_ENCODER_TICKS_PER_ROTATION * SLIDE_SPOOL_ROTATIONS_TO_MAX_EXTENSION) / 2.0));
            rightSlideMotor.setTargetPosition((int) ((SLIDE_ENCODER_TICKS_PER_ROTATION * SLIDE_SPOOL_ROTATIONS_TO_MAX_EXTENSION) / 2.0));
        } else if (mode == 2){
            leftSlideMotor.setTargetPosition((int) ((SLIDE_ENCODER_TICKS_PER_ROTATION * SLIDE_SPOOL_ROTATIONS_TO_MAX_EXTENSION) ));
            rightSlideMotor.setTargetPosition((int) ((SLIDE_ENCODER_TICKS_PER_ROTATION * SLIDE_SPOOL_ROTATIONS_TO_MAX_EXTENSION) ));
        }

    }
}