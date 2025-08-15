///*
//Steps to Upload FTCLib to OnBot Java
//
//Download FTCLib Source Files:
//Visit the FTCLib GitHub repository.
//
//Download the source code as a ZIP file or clone the repository:
//git clone https://github.com/FTCLib/FTCLib.git
//Extract the Necessary Classes:
//FTCLib is modular, so you don't need the entire library for odometry. Focus on these directories:
//geometry (for Pose2d, Rotation2d, etc.)
//kinematics (for HolonomicOdometry and related classes)
//Locate these files under src/main/java/com/arcrobotics/ftclib.
//Upload Classes to OnBot Java:
//Open the OnBot Java web interface on your Driver Station phone or computer.
//For each required file:
//Create a new Java class in OnBot Java (e.g., HolonomicOdometry.java).
//Copy and paste the contents of the corresponding FTCLib file into the new class.
//Ensure the package names at the top of each file match the folder structure, or adjust them to your needs (e.g., package org.firstinspires.ftc.teamcode).
//Adjust Imports and Package Names:
//Since FTCLib's original package structure (com.arcrobotics.ftclib) might not work in OnBot Java, you can change the package names of the uploaded files to org.firstinspires.ftc.teamcode.
//Update imports in your HolonomicOdometry-based program to match the new package structure.
//Use the Uploaded Library:
//Write your odometry code in OnBot Java, importing the classes you uploaded (e.g., HolonomicOdometry, Pose2d).
//
//Example:
//import org.firstinspires.ftc.teamcode.HolonomicOdometry;
//import org.firstinspires.ftc.teamcode.Pose2d;
//
//
// */
//
//
//
//
//package org.firstinspires.ftc.teamcode;
//
//import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.IMU;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
//import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.arcrobotics.ftclib.hardware.motors.MotorEx;
//
//
//@Autonomous(name = "Merged Auto: IMU and Movement", group = "Sensor")
//
//public class Auto_Movement extends LinearOpMode {
//
//    DcMotor backLeftDrive = null;
//    DcMotor frontLeftDrive = null;
//    DcMotor backRightDrive = null;
//    DcMotor frontRightDrive = null;
//    IMU imu;
//    private ElapsedTime runtime = new ElapsedTime();
//    private double currentHeading;
//    private double x;
//    private double y;
//    private DcMotor leftEncoder, rightEncoder, horizontalEncoder;
//    private HolonomicOdometry odometry;
//    private double prevX = 0;
//    private double prevY = 0;
//    private double distanceTravelled = 0;
//
//    /*
//
//
//    private static final double ODOMETRY_TICKS_PER_REVOLUTION = 2000;
//    private static final double ODOMETRY_POD_DIAMETER = 0.032;
//    private static final double ODOMETRY_POD_DISTANCE_PER_TICK = Math.PI * ODOMETRY_POD_DIAMETER / ODOMETRY_TICKS_PER_REVOLUTION;
//
//    */
//    private static final double TRACKWIDTH = 0.25; //Distance between left and right wheels
//
//    private static final double CENTER_WHEEL_OFFSET = 0.255; //FIND CORRECT VALUE Distance form center of the robot to the central odometry pod
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftMotor");
//        backRightDrive = hardwareMap.get(DcMotor.class, "backRightMotor");
//        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftMotor");
//        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightMotor");
//        imu = hardwareMap.get(IMU.class, "imu");
//
//        leftEncoder = hardwareMap.get(DcMotor.class, "leftEncoder");
//        rightEncoder = hardwareMap.get(DcMotor.class, "rightEncoder");
//        horizontalEncoder = hardwareMap.get(DcMotorEx.class, "horizontalEncoder");
//
//        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        horizontalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        horizontalEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        HolonomicOdometry odometry = new HolonomicOdometry(
//                () -> leftEncoder.getCurrentPosition(),
//                () -> rightEncoder.getCurrentPosition(),
//                () -> horizontalEncoder.getCurrentPosition(),
//                TRACKWIDTH,
//                CENTER_WHEEL_OFFSET
//        );
//
//        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
//        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
//
//        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
//
//        imu.initialize(new IMU.Parameters(orientationOnRobot));
//
//        // Loop and update the dashboard
//        while (!isStopRequested() && !opModeIsActive()) {
//
//            telemetry.addData("Hub orientation", "Logo=%s   USB=%s\n ", logoDirection, usbDirection);
//
//            // Check to see if heading reset is requested
//            //The commented code requires human input and is therefore only useful for DEBUGGING if we face an error in the testing of the code
//            // This is because Autonomous cannot use human inputs
//            /*if (gamepad1.y) {
//                telemetry.addData("Yaw", "Resetting\n");
//                imu.resetYaw();
//            } else {
//                telemetry.addData("Yaw", "Press Y (triangle) on Gamepad to reset\n");
//            }*/
//
//            // Retrieve Rotational Angles and Velocities
//            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
//            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
//
//            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
//            telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
//            telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
//            telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
//            telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
//            telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
//            telemetry.update();
//        }
//
//        /*final IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
//                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
//        imu.initialize(parameters);*/
//
//        waitForStart();
//        runtime.reset();
//
//        //double targetHeading = Math.toRadians(90);
//        //double power = 0.5;
//
//        while (opModeIsActive()) {
//
//            odometry.updatePose();
//
//            double x = odometry.getPose().getX();
//            double y = odometry.getPose().getY();
//            Heading();
//
//            double deltaX = x - prevX;
//            double deltaY = y - prevY;
//
//            //double currentHeading = odometry.getPose().getHeading();
//
//            /*if (Math.abs(targetHeading-currentHeading)<Math.toRadians(2)) {
//                stopDriving();
//                break;
//            }*/
//
//            double distanceThisIteration = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
//            distanceTravelled += distanceThisIteration;
//
//            prevX = x;
//            prevY = y;
//
//
//            telemetry.addData("Distance Travelled", distanceTravelled);
//            telemetry.addData("X Position", x);
//            telemetry.addData("Y Position", y);
//            telemetry.addData("Current Heading (degrees)", Math.toDegrees(currentHeading));
//
//            telemetry.update();
//
//            //ODOMETRY CODE TRY
//
//        }
//
//        ContinuousDistancing(0.75, y, x, true);
//        ContinuousDistancing(0.5, y, x, false);
//        //RobotRotation(90);
//        OdometricRotation(90, false);
//        ContinuousDistancing(1.6, y, x, true);
//        //RobotRotation(-90);
//        OdometricRotation(90, true);
//        ContinuousDistancing(0.32, y ,x, true);
//        for (int l = 0; l < 2; l++) {
//            ContinuousDistancing(1.6, y, x, false);
//            ContinuousDistancing(1.6, y, x, true);
//            ContinuousDistancing(0.32, y, x, true);
//        }
//        ContinuousDistancing(1.6, y, x, false);
//        ContinuousDistancing(3.6, y, x, false);
//
//        /*
//        USE CODE TO CONFIRM STRAFING RIGHT IS POSITIVE VALUE AND OPPOSITE FOR LEFT
//
//        telemetry.addData("Y Position", odometry.getPose().getY());
//        telemetry.update();
//         */
///*
//        //ORIGINAL TIMEBASE CODE --> BETTER VERSION DONE IN ANOTHER JAVA DOC
//
//            //Simplest Auto Code for scoring one sample and parking
//            drive(1, 0, 0);
//            sleep(2000);
//            stopDriving();
//            drive(-1, 0, 0);
//            sleep(8000);
//            stopDriving();
//
//            //Medium Auto Code for trying to score ONE more neutral samples
//            //Right --> -ve?  Left--> +ve? for lateral (y-axis) movement
//            drive(1, 0, 0);
//            sleep(2000);
//            stopDriving();
//            drive(-0.25, 0, 0);
//            sleep(2000);
//            //RobotRotation(90);  only use IMU code if needed
//            //***TEST IMU code before the day, just in case***
//            drive(0, -1, 0);
//            sleep(6000);
//            drive(0.25, 0, 0);
//            sleep(1000);
//            drive(0, 1, 0);
//            sleep(6000);
//            stopDriving();
//
//        */
//
//    }
//
//    private void stopDriving() {
//        backLeftDrive.setPower(0);
//        frontLeftDrive.setPower(0);
//        backRightDrive.setPower(0);
//        backLeftDrive.setPower(0);
//    }
//
//    private void drive(double x1, double y1, double r) {
//        double leftFrontPower = x1 + y1 + r;
//        double rightFrontPower = x1 - y1 - r;
//        double leftBackPower = x1 - y1 + r;
//        double rightBackPower = x1 + y1 - r;
//        double max;
//
//        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
//        max = Math.max(max, Math.abs(leftBackPower));
//        max = Math.max(max, Math.abs(rightBackPower));
//
//        if (max > 1.0) {
//            leftFrontPower /= max;
//            rightFrontPower /= max;
//            leftBackPower /= max;
//            rightBackPower /= max;
//        }
//
//        frontLeftDrive.setPower(leftFrontPower);
//        frontRightDrive.setPower(rightFrontPower);
//        backLeftDrive.setPower(leftBackPower);
//        backRightDrive.setPower(rightBackPower);
//
//        telemetry.addData("Left Front Power", leftFrontPower);
//        telemetry.addData("Right Front Power", rightFrontPower);
//        telemetry.addData("Back Left Power", leftBackPower);
//        telemetry.addData("Back Right Power", rightBackPower);
//        telemetry.update();
//    }
//    /*
//    IMU CODE
//
//    private double yaw() {
//        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//        return yaw;
//    }
//
//    private void RobotRotation(double targetYaw) {
//        double initialYaw = yaw();
//        double currentYaw = initialYaw;
//        drive(0, 0, 0.5);
//        while (currentYaw - initialYaw <= targetYaw ) {
//            // turn till
//            currentYaw = yaw();
//        }
//    }
//*/
//    private void ContinuousDistancing(double targetDistance, double Cy, double Cx, boolean positiveMove) {
//        //targetDistance is where you want to reach at the end
//        double initialDistance = distanceTravelled;
//        double currentDistance = initialDistance;
//        while (Math.abs(currentDistance - initialDistance) < targetDistance ) {
//            // move forward
//            if (Cy==0) {
//                currentDistance = distanceTravelled;
//                if (positiveMove == true) {
//                    drive(1, 0, 0);
//                } else {
//                    drive(-1,0,0);
//                }
//            } else if (Cx==0) {
//                currentDistance = distanceTravelled;
//                if (positiveMove == true) {
//                    drive(0, 1, 0);
//                } else {
//                    drive(0,-1,0);
//                }
//            }
//        }
//        stopDriving();
//    }
//
//    private void OdometricRotation(double targetHeading, boolean left) {
//        double initialHeading = currentHeading;
//        odometry.updatePose();
//        if (left == true) {
//            drive(0,0, -1);
//            if (Math.abs(Math.toRadians(targetHeading)-currentHeading)<Math.toRadians(2)){
//                stopDriving();
//            }
//        } else {
//            drive(0,0, 1);
//            if (Math.abs(Math.toRadians(targetHeading)-currentHeading)<Math.toRadians(2)){
//                stopDriving();
//            }
//        }
//
//    }
//
//    private void Heading() {
//        currentHeading = odometry.getPose().getHeading();
//    }
//
//
//
//}
