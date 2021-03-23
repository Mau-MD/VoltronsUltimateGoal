package org.firstinspires.ftc.teamcode.Voltrons.OpMode.Autonomous.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Voltrons.hardware.Belt;
import org.firstinspires.ftc.teamcode.Voltrons.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Voltrons.hardware.Intake;
import org.firstinspires.ftc.teamcode.Voltrons.hardware.Launcher;
import org.firstinspires.ftc.teamcode.Voltrons.hardware.WoobleArm;

@Config
@TeleOp(name="Drive Encoder Gyro Test", group="Test")
public class DriveEncoderGyroTest extends LinearOpMode {

    public static double wobbleHandMin = 0;
    public static double wobbleHandMax = 180; // Degrees
    public static double wobbleArmMin = 0;
    public static double wobbleArmMax = 270;

    public static PIDFCoefficients wArm = new PIDFCoefficients(0,0,0,0);
    public static PIDFCoefficients turn = new PIDFCoefficients(0,0,0,0);
    public static PIDFCoefficients gyro = new PIDFCoefficients(0,0,0,0);
    public static PIDFCoefficients encoder = new PIDFCoefficients(0.0007,0,0.001,0);


    @Override
    public void runOpMode() {
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


        PIDFController wobblePIDF = new PIDFController(wArm.p, wArm.i, wArm.d, wArm.f);
        PIDFController gyroPIDF = new PIDFController(gyro.p, gyro.i, gyro.d, gyro.f);
        PIDFController turnPIDF = new PIDFController(turn.p, turn.i, turn.d, turn.f);
        PIDFController encoderPIDF = new PIDFController(encoder.p, encoder.i, encoder.d, encoder.f);

        Drivetrain drive = new Drivetrain(frontLeft, frontRight, backLeft, backRight, imu);
        Belt belt = new Belt(beltDown, betlUp);
        Intake intake = new Intake(intakeMotor);
        Launcher launcher = new Launcher(leftLauncher, rightLauncher);
        WoobleArm woobleArm = new WoobleArm(wobbleArm, wobbleHand, wobblePIDF, wobbleArmMin, wobbleArmMax, 340);

        ElapsedTime aButton = new ElapsedTime();
        aButton.reset();

        drive.setDashboard(FtcDashboard.getInstance());

        waitForStart();

        // TODO: Separate Front and Strafe. Time if anything goes wrong. Add set orientation every movement.
        // TODO: Different PIDF for strafing?
        while (opModeIsActive()) {

            turnPIDF.setPIDF(turn.p, turn.i, turn.d, turn.f);
            gyroPIDF.setPIDF(gyro.p, gyro.i, gyro.d, gyro.f);
            encoderPIDF.setPIDF(encoder.p, encoder.i, encoder.d, encoder.f);

            if (gamepad1.a && aButton.milliseconds() > 100) {
                drive.driveEncoderGyro(new double[] {1,1,1,1}, 180, 100, gyroPIDF, encoderPIDF);
            } else if (gamepad1.b && aButton.milliseconds() > 100) {
                drive.driveEncoderGyro(new double[] {-1,-1,-1,-1}, 180, -100, gyroPIDF, encoderPIDF);
            } else if (gamepad1.x && aButton.milliseconds() > 100) {
                drive.driveEncoderGyro(new double[] {1,-1,-1,1}, 180, 100, gyroPIDF, encoderPIDF);
            } else if (gamepad1.y && aButton.milliseconds() > 100) {
                drive.driveEncoderGyro(new double[] {-1,1,1,-1}, 180, 100, gyroPIDF, encoderPIDF);
            }
        }
    }
}