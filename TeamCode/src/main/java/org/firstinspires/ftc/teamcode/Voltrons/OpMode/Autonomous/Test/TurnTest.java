package org.firstinspires.ftc.teamcode.Voltrons.OpMode.Autonomous.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Voltrons.Path.Point;
import org.firstinspires.ftc.teamcode.Voltrons.Path.Spline;
import org.firstinspires.ftc.teamcode.Voltrons.control.PID;
import org.firstinspires.ftc.teamcode.Voltrons.control.PIDICoeff;
import org.firstinspires.ftc.teamcode.Voltrons.hardware.Belt;
import org.firstinspires.ftc.teamcode.Voltrons.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Voltrons.hardware.Intake;
import org.firstinspires.ftc.teamcode.Voltrons.hardware.Launcher;
import org.firstinspires.ftc.teamcode.Voltrons.hardware.WoobleArm;

@Config
@TeleOp(name="Turn Test", group="Tests")
public class TurnTest extends LinearOpMode {

    public static double wobbleHandMin = 0;
    public static double wobbleHandMax = 180; // Degrees
    public static double wobbleArmMin = 0;
    public static double wobbleArmMax = 270;

    public static PIDICoeff armCoeff = new PIDICoeff(0.0008,0,0.0001, 0);
    public static PIDICoeff turnCoeff = new PIDICoeff(0.07,0, 0,0);
    public static PIDICoeff turnCoeff2 = new PIDICoeff(0.04,0.02, 0.01,30);
    public static PIDICoeff encoderCoeff = new PIDICoeff(0,0,0,0);
    public static double gyroKp = 0;

    public static double iLimit = 30;

    @Override
    public void runOpMode()
    {
        // Drivetrain Motors
        Motor frontLeft = new Motor(hardwareMap, "fl", Motor.GoBILDA.RPM_117);
        Motor frontRight = new Motor(hardwareMap, "fr", Motor.GoBILDA.RPM_117);
        Motor backLeft = new Motor(hardwareMap, "bl", Motor.GoBILDA.RPM_117);
        Motor backRight= new Motor(hardwareMap, "br", Motor.GoBILDA.RPM_117);

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
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
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


        PID wobblePID = new PID(armCoeff);
        PID turnPID = new PID(turnCoeff);
        PID encoderPID = new PID(encoderCoeff);


        Drivetrain drive = new Drivetrain(frontLeft, frontRight, backLeft, backRight, imu);
        Belt belt = new Belt(beltDown, betlUp);
        Intake intake = new Intake(intakeMotor);
        Launcher launcher = new Launcher(leftLauncher, rightLauncher);
        WoobleArm woobleArm = new WoobleArm(wobbleArm, wobbleHand, wobblePID, wobbleArmMin, wobbleArmMax, 340);

        ElapsedTime aButton = new ElapsedTime();
        aButton.reset();

        drive.setDashboard(FtcDashboard.getInstance());

        waitForStart();

        while(opModeIsActive())
        {
            // Pull request
            turnPID.setCoeff(turnCoeff);
            drive.setOrientationPID(turnPID);

            if (gamepad1.a && aButton.milliseconds() > 100) {
                drive.setOrientation(0.8,270);
                aButton.reset();
            }
            else if (gamepad1.b && aButton.milliseconds() > 100) {
                drive.setOrientation(0.8,180);
                aButton.reset();
            }
            else if (gamepad1.x && aButton.milliseconds() > 100) {
                drive.setOrientationC(0.8, 270);
            }
            else if (gamepad1.y && aButton.milliseconds() > 100){
                drive.setOrientationC(0.8, 180);
            }
        }
    }
}

