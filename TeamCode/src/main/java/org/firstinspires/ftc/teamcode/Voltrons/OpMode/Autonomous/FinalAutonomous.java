package org.firstinspires.ftc.teamcode.Voltrons.OpMode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Voltrons.Path.Point;
import org.firstinspires.ftc.teamcode.Voltrons.Path.Spline;
import org.firstinspires.ftc.teamcode.Voltrons.Vision.RingPipeline;
import org.firstinspires.ftc.teamcode.Voltrons.control.PID;
import org.firstinspires.ftc.teamcode.Voltrons.control.PIDICoeff;
import org.firstinspires.ftc.teamcode.Voltrons.hardware.Belt;
import org.firstinspires.ftc.teamcode.Voltrons.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Voltrons.hardware.Intake;
import org.firstinspires.ftc.teamcode.Voltrons.hardware.Launcher;
import org.firstinspires.ftc.teamcode.Voltrons.hardware.WoobleArm;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import java.sql.Driver;

public class FinalAutonomous extends LinearOpMode {

    public static double wobbleHandMin = 0;
    public static double wobbleHandMax = 180; // Degrees
    public static double wobbleArmMin = 0;
    public static double wobbleArmMax = 726;

    public static double A = 50;
    public static double B = 650;
    public static double C = 300;

    public static PIDICoeff armCoeff = new PIDICoeff(0.0008,0.0003,0,500);
    public static PIDICoeff turnCoeff = new PIDICoeff(0.07,0, 0,0);
    public static double gyroKp = 0.01;

    OpenCvCamera phoneCam;
    RingPipeline visionPipeline;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.openCameraDevice();

        visionPipeline = new RingPipeline();
        phoneCam.setPipeline(visionPipeline);
        phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);

        FtcDashboard.getInstance().startCameraStream(phoneCam,60);

        // Drivetrain Motors
        Motor frontLeft = new Motor(hardwareMap, "fl", Motor.GoBILDA.RPM_117);
        Motor frontRight = new Motor(hardwareMap, "fr", Motor.GoBILDA.RPM_117);
        Motor backLeft = new Motor(hardwareMap, "bl", Motor.GoBILDA.RPM_117);
        Motor backRight = new Motor(hardwareMap, "br", Motor.GoBILDA.RPM_117);

        // Wobble Motors
        Motor wobbleArm = new Motor(hardwareMap, "wa");
        SimpleServo wobbleHand = new SimpleServo(hardwareMap, "wh", wobbleHandMin, wobbleHandMax);

        // Intake and movement Motors
        Motor intakeMotor = new Motor(hardwareMap, "in");
        CRServo beltDown = hardwareMap.crservo.get("bd");
        CRServo betlUp = hardwareMap.crservo.get("bu");

        // Launcher Motors
        Motor leftLauncher = new Motor(hardwareMap, "ll", Motor.GoBILDA.RPM_312);
        Motor rightLauncher = new Motor(hardwareMap, "rl", Motor.GoBILDA.RPM_312);

        // Imu
        BNO055IMU imu;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Configure Drivetrain Motors
        frontLeft.setRunMode(Motor.RunMode.RawPower);
        frontRight.setRunMode(Motor.RunMode.RawPower);
        backLeft.setRunMode(Motor.RunMode.RawPower);
        backRight.setRunMode(Motor.RunMode.RawPower);

        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        frontLeft.setInverted(true);
        frontRight.setInverted(true);
        backLeft.setInverted(false);
        backRight.setInverted(false);

        wobbleArm.resetEncoder();
        PID wobblePID = new PID(armCoeff);
        PID turnPID = new PID(turnCoeff);

        Drivetrain drive = new Drivetrain(frontLeft, frontRight, backLeft, backRight, imu);
        Belt belt = new Belt(beltDown, betlUp);
        Intake intake = new Intake(intakeMotor);
        Launcher launcher = new Launcher(leftLauncher, rightLauncher);
        WoobleArm arm = new WoobleArm(wobbleArm, wobbleHand, wobblePID, wobbleArmMin, wobbleArmMax, 340);

        drive.setDashboard(FtcDashboard.getInstance());

        arm.closeHand();
        arm.setGoal(0);
        arm.setPIDI(armCoeff);
        arm.setArmRange(wobbleArmMin, wobbleArmMax);
        arm.setRange(wobbleHandMin, wobbleHandMax);

        drive.setOrientationPID(turnPID);
        drive.setGyroKp(gyroKp);

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        waitForStart();


        if (opModeIsActive()) {

            Point[] first = new Point[]{
                    new Point(166, 56),
                    new Point(175, 89),
                    new Point(177, 95),
                    new Point(210, 150),
                    new Point(220, 170)
            };

            Spline firstSpline = new Spline(first, 5, 5);
            drive.followPath(firstSpline, 0.5);

            sleep(1000);

            drive.setOrientation(0.8, 0);

            sleep(1000);

            arm.resetSum();
            arm.setGoal(B);
            timer.reset();

            while (timer.milliseconds() < 5000) {
                arm.goToGoal();
            }

            arm.arm.set(0);

            arm.openHand();

            sleep(2000);

            drive.driveEncoderGyro(new double[]{-0.5, 0.5, 0.5, -10.5}, 0, 30, 0.1);

            sleep(1000);

            drive.setOrientation(0.8, 180);

            sleep(1000);

            Point[] second = new Point[]{
                    new Point(120, 60),
                    new Point(150, 50),
                    new Point(185, 63),
                    new Point(205, 150),
                    new Point(210, 190)
            };

            Spline secondSpline = new Spline(second, 5, 5);
            drive.followPathReverse(secondSpline, 0.5);


            sleep(1000);

            arm.closeHand();

            sleep(2000);

            arm.resetSum();
            arm.setGoal(A);
            timer.reset();

            while (timer.milliseconds() < 5000) {
                arm.goToGoal();
            }

            arm.arm.set(0);

            sleep(1000);

            drive.followPath(secondSpline, 0.5);

            sleep(1000);

            drive.setOrientation(0.8, 0);

            sleep(1000);

            arm.resetSum();
            arm.setGoal(B);
            timer.reset();

            while (timer.milliseconds() < 5000) {
                arm.goToGoal();
            }

            arm.arm.set(0);

            arm.openHand();

            sleep(2000);

            drive.driveEncoderGyro(new double[]{-0.5, 0.5, 0.5, -10.5}, 0, 30, 0.1);


        }
    }

}
