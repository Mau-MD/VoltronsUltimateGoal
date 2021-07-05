package org.firstinspires.ftc.teamcode.Voltrons.OpMode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

@Config
@Autonomous(name="Auto v2", group="UO")
public class Autonomousv2 extends LinearOpMode {

    public static double wobbleHandMin = 0;
    public static double wobbleHandMax = 180; // Degrees
    public static double wobbleArmMin = 0;
    public static double wobbleArmMax = 726;

    public static double A = 50;
    public static double B = 650;
    public static double C = 300;

    public static PIDICoeff armCoeff = new PIDICoeff(0.0008,0.0003,0,500);
    public static PIDICoeff turnCoeff = new PIDICoeff(0.07,0, 0,0);
    public static PIDICoeff encoderCoeff = new PIDICoeff(0.0001,0,0,0);
    public static double gyroKp = 0.05;

    public static double power1 = 0.9;
    public static double power2 = 0.9;

    public static double distanceA = 140;
    public static double distanceB = 20;
    public static double distanceZeroRings = 20;
    public static double distanceOneRing = 100;
    public static double distanceFourRings = 150;

    public static double launcPower = 0.35;
    public static long launchTime = 7000;

    DcMotor left_launcher;
    DcMotor right_launcher;
    DcMotor intake;

    CRServo belt_down;
    CRServo belt_up;

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

        // Drivetrain Motor
        Motor frontLeft = new Motor(hardwareMap, "fl", 537.6, 340);
        Motor frontRight = new Motor(hardwareMap, "fr", 537.6,340);
        Motor backLeft = new Motor(hardwareMap, "bl", 537.6,340);
        Motor backRight = new Motor(hardwareMap, "br", 537.6,340);

        // Wobble Motors
        Motor wobbleArm = new Motor(hardwareMap, "wa");
        SimpleServo wobbleHand = new SimpleServo(hardwareMap, "wh", wobbleHandMin, wobbleHandMax);

        // Intake and movement Motors
        Motor intakeMotor = new Motor(hardwareMap, "in");


        belt_down = hardwareMap.crservo.get("bd");
        belt_up = hardwareMap.crservo.get("bu");

        // Launcher Motors
        Motor leftLauncher = new Motor(hardwareMap, "ll");
        Motor rightLauncher = new Motor(hardwareMap, "rl");

        // 1
        right_launcher = hardwareMap.dcMotor.get("rl");
        // 0
        left_launcher = hardwareMap.dcMotor.get("ll");


        intake = hardwareMap.dcMotor.get("in");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        right_launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setInverted(true);
        frontRight.setInverted(true);
        backLeft.setInverted(false);
        backRight.setInverted(false);

        wobbleArm.resetEncoder();
        wobbleArm.setInverted(true);

        PID wobblePID = new PID(armCoeff);
        PID turnPID = new PID(turnCoeff);
        PID encoderPID = new PID(encoderCoeff);

        Drivetrain drive = new Drivetrain(frontLeft, frontRight, backLeft, backRight, imu);
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
        drive.setEncoderPID(encoderPID);

        drive.setDashboard(FtcDashboard.getInstance());
        arm.setDashboard(FtcDashboard.getInstance());

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        drive.setOrientationPID(turnPID);


        waitForStart();

        int rings;

        if (opModeIsActive()) {

            if (visionPipeline.ring1 == 0 && visionPipeline.ring4 == 0) {
                rings = 4;
            }
            else if (visionPipeline.ring1 == 0) {
                rings = 1;
            }
            else {
                rings = 0;
            }

            left_launcher.setPower(launcPower);
            right_launcher.setPower(-launcPower);

            drive.driveEncoderSimple(new double[]{power1, power1, power1, power1}, distanceA);
            drive.driveEncoderSimple(new double[]{power1, -power1, -power1, power1}, distanceB);

            drive.idle();

            sleep(500);

            // Corregir Orientacion
            drive.setOrientation(0.8, 180);

            sleep(1500);

            // Prender belt e intake

            belt_down.setPower(1);
            belt_up.setPower(1);
            intake.setPower(-1);

            timer.reset();
            while (timer.milliseconds() < launchTime) {
                telemetry.addLine("im shooting lol");
                telemetry.update();
            }

            belt_down.setPower(0);
            belt_up.setPower(0);
            intake.setPower(0);
            left_launcher.setPower(0);
            right_launcher.setPower(0);

            drive.driveEncoderSimple(new double[]{power1, -power1, -power1, power1}, distanceB);

            if (rings == 0) {
                drive.driveEncoderSimple(new double[]{power1, power1, power1, power1}, distanceZeroRings);
                drive.setOrientation(0.8, 0);
                drive.idle();

                arm.resetSum();
                arm.setGoal(650);
                timer.reset();

                while (timer.milliseconds() < 3000) {
                    arm.goToGoal();
                }

                arm.arm.set(0);

                arm.openHand();

                sleep(2000);

                drive.idle();

                // Se ira a la izquierda
                drive.driveEncoderSimple(new double[]{power1, -power1, -power1, power1}, distanceB);

            }
            else if (rings == 1) {
                drive.driveEncoderSimple(new double[]{power1, power1, power1, power1}, distanceOneRing);

                drive.idle();

                arm.resetSum();
                arm.setGoal(650);
                timer.reset();

                while (timer.milliseconds() < 3000) {
                    arm.goToGoal();
                }

                arm.arm.set(0);

                arm.openHand();

                sleep(2000);

                drive.idle();

                // Se ira a la izquierda
                drive.driveEncoderSimple(new double[]{power1, -power1, -power1, power1}, distanceB);
                drive.driveEncoderSimple(new double[]{-power1, -power1, -power1, -power1}, distanceOneRing - 20);

            }
            else {
                drive.driveEncoderSimple(new double[]{power1, power1, power1, power1}, distanceFourRings);
                drive.setOrientation(0.8, 0);
                drive.idle();

                arm.resetSum();
                arm.setGoal(650);
                timer.reset();

                while (timer.milliseconds() < 3000) {
                    arm.goToGoal();
                }

                arm.arm.set(0);

                arm.openHand();

                sleep(2000);

                drive.idle();

                // Se ira a la izquierda
                drive.driveEncoderSimple(new double[]{power1, -power1, -power1, power1}, distanceB);
                drive.driveEncoderSimple(new double[]{power1, power1, power1, power1}, distanceFourRings - 80);
                drive.driveEncoderSimple(new double[]{-power1, power1, power1, -power1}, distanceB);
                drive.driveEncoderSimple(new double[]{power1, power1, power1, power1}, 40);

            }

            drive.idle();
        }
    }

}
