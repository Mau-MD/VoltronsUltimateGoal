package org.firstinspires.ftc.teamcode.Voltrons.OpMode.Autonomous.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Voltrons.Vision.RingPipeline;
import org.firstinspires.ftc.teamcode.Voltrons.control.PIDICoeff;
import org.firstinspires.ftc.teamcode.Voltrons.hardware.Imu;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;


@Config
@TeleOp(name = "Encoder Autonomus", group = "Test")
public class AutonomousEncoder extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    DcMotor left_launcher;
    DcMotor right_launcher;
    DcMotor intake;

    CRServo belt;

    OpenCvCamera phoneCam;
    RingPipeline visionPipeline;

    Imu imu;
    int rings;

    Motor arm;
    Servo hand;

    public static int downPosition = 500;
    public static int upPosition = 0;
    public static double power = 0.5;


    // Move Variables
    public static int A = 2500;
    public static int B = 1000;
    public static int C = 1000;
    public static int D = 500;
    public static int E = 2500;
    public static int F= 2500;

    public static double p = 0.001;
    public static double corrDir = 1;
    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.openCameraDevice();

        visionPipeline = new RingPipeline();
        phoneCam.setPipeline(visionPipeline);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);

        FtcDashboard.getInstance().startCameraStream(phoneCam, 60);

        frontLeft = hardwareMap.dcMotor.get("fl");
        frontRight = hardwareMap.dcMotor.get("fr");
        backLeft = hardwareMap.dcMotor.get("bl");
        backRight = hardwareMap.dcMotor.get("br");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_launcher = hardwareMap.dcMotor.get("ll");
        right_launcher = hardwareMap.dcMotor.get("rl");

        right_launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        belt = hardwareMap.crservo.get("bu");
        belt.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.dcMotor.get("in");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU _imu;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        _imu = hardwareMap.get(BNO055IMU.class, "imu");
        _imu.initialize(parameters);

        imu = new Imu(_imu);


        hand = hardwareMap.servo.get("hand");
        arm = new Motor(hardwareMap, "arm", Motor.GoBILDA.RPM_117);

        int target = 0;
        boolean open = false;

        waitForStart();

        if (opModeIsActive()) {

            if (visionPipeline.ring1 == 0 && visionPipeline.ring4 == 0) {
                rings = 4;
            } else if (visionPipeline.ring1 == 0) {
                rings = 1;
            } else {
                rings = 0;
            }

            front(2500, 0.7, 180);
            sleep(1000);
            back(2500, 0.7, 180);
            sleep(1000);
            left(2500, 0.7, 180);
            sleep(1000);
            right(2500, 0.7, 180);
//            // It starts
//            launch(0.30);
//            front(A, 0.7);
//            // Launch
//            front(B, 0.7);
//            turnTo(90, 0.7);
//            back(C, 0.7);
//            // Wobble
//            right(D, 0.7);
//            front(E, 0.7);
//            left(D, 0.7);



        }
    }

    public void front(int ticks, double power) {

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(ticks);
        backLeft.setTargetPosition(ticks);
        backRight.setTargetPosition(ticks);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while (opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            telemetry.addData("frontLeft", frontLeft.getCurrentPosition());
            telemetry.addData("frontRight", frontRight.getCurrentPosition());
            telemetry.addData("backLeft", backLeft.getCurrentPosition());
            telemetry.addData("backRight", backRight.getCurrentPosition());
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void front(int ticks, double power, double angle) {

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(ticks);
        backLeft.setTargetPosition(ticks);
        backRight.setTargetPosition(ticks);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);


        while (opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            //error > 0 entonces girar a la derecha

            double error = Imu.getError(imu.getAngleNormalized(), angle);
            double correction = error * p * corrDir;

            frontLeft.setPower(power + correction);
            frontRight.setPower(power - correction);
            backLeft.setPower(power + correction);
            backRight.setPower(power - correction);

            telemetry.addData("frontLeft", frontLeft.getCurrentPosition());
            telemetry.addData("frontRight", frontRight.getCurrentPosition());
            telemetry.addData("backLeft", backLeft.getCurrentPosition());
            telemetry.addData("backRight", backRight.getCurrentPosition());
            telemetry.addData("error", error);
            telemetry.addData("correction", correction);
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void back(int ticks, double power) {

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(-ticks);
        frontRight.setTargetPosition(-ticks);
        backLeft.setTargetPosition(-ticks);
        backRight.setTargetPosition(-ticks);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(-power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(-power);

        while (opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            telemetry.addData("frontLeft", frontLeft.getCurrentPosition());
            telemetry.addData("frontRight", frontRight.getCurrentPosition());
            telemetry.addData("backLeft", backLeft.getCurrentPosition());
            telemetry.addData("backRight", backRight.getCurrentPosition());
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void back(int ticks, double power, double angle) {

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(-ticks);
        frontRight.setTargetPosition(-ticks);
        backLeft.setTargetPosition(-ticks);
        backRight.setTargetPosition(-ticks);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(-power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(-power);

        while (opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            double error = Imu.getError(imu.getAngleNormalized(), angle);
            double correction = error * p * corrDir;

            frontLeft.setPower(power + correction);
            frontRight.setPower(power - correction);
            backLeft.setPower(power + correction);
            backRight.setPower(power - correction);

            telemetry.addData("frontLeft", frontLeft.getCurrentPosition());
            telemetry.addData("frontRight", frontRight.getCurrentPosition());
            telemetry.addData("backLeft", backLeft.getCurrentPosition());
            telemetry.addData("backRight", backRight.getCurrentPosition());
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void left(int ticks, double power) {

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(-ticks);
        frontRight.setTargetPosition(ticks);
        backLeft.setTargetPosition(ticks);
        backRight.setTargetPosition(-ticks);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(-power);

        while (opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            telemetry.addData("frontLeft", frontLeft.getCurrentPosition());
            telemetry.addData("frontRight", frontRight.getCurrentPosition());
            telemetry.addData("backLeft", backLeft.getCurrentPosition());
            telemetry.addData("backRight", backRight.getCurrentPosition());
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void left(int ticks, double power, double angle) {

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(-ticks);
        frontRight.setTargetPosition(ticks);
        backLeft.setTargetPosition(ticks);
        backRight.setTargetPosition(-ticks);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(-power);

        while (opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            double error = Imu.getError(imu.getAngleNormalized(), angle);
            double correction = error * p * corrDir;

            frontLeft.setPower(power + correction);
            frontRight.setPower(power - correction);
            backLeft.setPower(power + correction);
            backRight.setPower(power - correction);

            telemetry.addData("frontLeft", frontLeft.getCurrentPosition());
            telemetry.addData("frontRight", frontRight.getCurrentPosition());
            telemetry.addData("backLeft", backLeft.getCurrentPosition());
            telemetry.addData("backRight", backRight.getCurrentPosition());
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void right(int ticks, double power) {

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(-ticks);
        backLeft.setTargetPosition(-ticks);
        backRight.setTargetPosition(ticks);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(-power);

        while (opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            telemetry.addData("frontLeft", frontLeft.getCurrentPosition());
            telemetry.addData("frontRight", frontRight.getCurrentPosition());
            telemetry.addData("backLeft", backLeft.getCurrentPosition());
            telemetry.addData("backRight", backRight.getCurrentPosition());
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void right(int ticks, double power, double angle) {

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(-ticks);
        backLeft.setTargetPosition(-ticks);
        backRight.setTargetPosition(ticks);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(-power);

        while (opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            double error = Imu.getError(imu.getAngleNormalized(), angle);
            double correction = error * p * corrDir;

            frontLeft.setPower(power + correction);
            frontRight.setPower(power - correction);
            backLeft.setPower(power + correction);
            backRight.setPower(power - correction);

            telemetry.addData("frontLeft", frontLeft.getCurrentPosition());
            telemetry.addData("frontRight", frontRight.getCurrentPosition());
            telemetry.addData("backLeft", backLeft.getCurrentPosition());
            telemetry.addData("backRight", backRight.getCurrentPosition());
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void turnRight(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(power);
        backRight.setPower(-power);
    }

    public void turnLeft(double power) {
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(-power);
        backRight.setPower(power);
    }

    public void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void turnTo(double angle, double power) {

        double error = Imu.getError(imu.getAngleNormalized(), angle);
        if (error < 0) {
            while (error < 0 && opModeIsActive()) {
                error = Imu.getError(imu.getAngleNormalized(), angle);
                turnRight(power);
                if (error > -10) {
                    power /= 2.0;
                }
                telemetry.addData("error", error);
                telemetry.update();
            }
            stopMotors();
            error = Imu.getError(imu.getAngleNormalized(), angle);
            while (error > 0 && opModeIsActive()) {
                error = Imu.getError(imu.getAngleNormalized(), angle);
                turnLeft(0.1);
                telemetry.addData("error", error);
                telemetry.update();
            }
            stopMotors();

        } else if (error > 0) {
            while (error > 0 && opModeIsActive()) {
                error = Imu.getError(imu.getAngleNormalized(), angle);
                turnLeft(power);
                if (error < 10) {
                    power /= 2.0;
                }
                telemetry.addData("error", error);
                telemetry.update();
            }
            stopMotors();
            error = Imu.getError(imu.getAngleNormalized(), angle);
            while (error < 0 && opModeIsActive()) {
                error = Imu.getError(imu.getAngleNormalized(), angle);
                turnRight(0.1);
                telemetry.addData("error", error);
                telemetry.update();
            }
            stopMotors();
        }
        stopMotors();
    }

    public void launch(double power) {
        left_launcher.setPower(-power);
        right_launcher.setPower(power);
    }

    public void stopLaunching() {
        left_launcher.setPower(0);
        right_launcher.setPower(0);
    }

    public void intakeBelt() {
        intake.setPower(-1);
        belt.setPower(1);
    }

    public void stopIntakeBelt() {
        intake.setPower(0);
        belt.setPower(0);
    }


    public void moveArmTo(int target) {

        arm.setTargetPosition(target);
        if (gamepad1.x) {
            double error = target - arm.getCurrentPosition();
            if (error > 0) {
                arm.set(power);
                while (error > 0 && opModeIsActive()) {
                    error = target - arm.getCurrentPosition();
                    telemetry.addData("Position", arm.getCurrentPosition());
                    telemetry.addData("target", target);
                    telemetry.update();
                }
                arm.set(0);
            }
            if (error < 0) {
                arm.set(-power);
                while (error < 0 && opModeIsActive()) {
                    error = target - arm.getCurrentPosition();
                    telemetry.addData("Position", arm.getCurrentPosition());
                    telemetry.addData("target", target);
                    telemetry.update();
                }
                arm.set(0);
            }
        }
        telemetry.addData("target", target);
        telemetry.addData("Position", arm.getCurrentPosition());
        telemetry.update();
    }
}


