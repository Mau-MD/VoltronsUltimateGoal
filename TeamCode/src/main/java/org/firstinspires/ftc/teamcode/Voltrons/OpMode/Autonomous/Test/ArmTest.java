package org.firstinspires.ftc.teamcode.Voltrons.OpMode.Autonomous.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Voltrons.control.PID;
import org.firstinspires.ftc.teamcode.Voltrons.control.PIDICoeff;
import org.firstinspires.ftc.teamcode.Voltrons.hardware.Belt;
import org.firstinspires.ftc.teamcode.Voltrons.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Voltrons.hardware.Intake;
import org.firstinspires.ftc.teamcode.Voltrons.hardware.Launcher;
import org.firstinspires.ftc.teamcode.Voltrons.hardware.WoobleArm;

@Config
@TeleOp(name="ArmTest", group = "test")
public class ArmTest extends LinearOpMode {

    public static double wobbleHandMin = 0;
    public static double wobbleHandMax = 180; // Degrees
    public static double wobbleArmMin = 0;
    public static double wobbleArmMax = 726;

    public static double A = 50;
    public static double B = 650;
    public static double C = 300;

    public static PIDICoeff armCoeff = new PIDICoeff(0.0008,0,0.0001,0);

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

        wobbleArm.resetEncoder();
        PID wobblePID = new PID(armCoeff);

        Drivetrain drive = new Drivetrain(frontLeft, frontRight, backLeft, backRight, imu);
        Belt belt = new Belt(beltDown, betlUp);
        Intake intake = new Intake(intakeMotor);
        Launcher launcher = new Launcher(leftLauncher, rightLauncher);
        WoobleArm arm = new WoobleArm(wobbleArm, wobbleHand, wobblePID, wobbleArmMin, wobbleArmMax, 340);
        ElapsedTime aButton = new ElapsedTime();

        arm.closeHand();
        aButton.reset();
        drive.setDashboard(FtcDashboard.getInstance());


        waitForStart();


        arm.setDashboard(FtcDashboard.getInstance());
        arm.setGoal(0);


        while (opModeIsActive()) {

            arm.setPIDI(armCoeff);
            arm.setArmRange(wobbleArmMin, wobbleArmMax);
            arm.setRange(wobbleHandMin, wobbleHandMax);

            if (gamepad1.a && aButton.milliseconds() > 40) {
                arm.setGoal(A);
                arm.resetSum();
                aButton.reset();
            }
            if (gamepad1.b && aButton.milliseconds() > 40) {
                arm.setGoal(B);
                arm.resetSum();
                aButton.reset();
            }
            if (gamepad1.x && aButton.milliseconds() > 40) {
                arm.setGoal(C);
                arm.resetSum();
                aButton.reset();
            }

            if (gamepad1.dpad_left && aButton.milliseconds() > 40) {
                arm.closeHand();
                aButton.reset();
            }
            if (gamepad1.dpad_right && aButton.milliseconds() > 40) {
                arm.openHand();
                aButton.reset();
            }


            arm.goToGoal();


        }
    }
}
