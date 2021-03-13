package org.firstinspires.ftc.teamcode.Voltrons.OpMode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Voltrons.hardware.Belt;
import org.firstinspires.ftc.teamcode.Voltrons.hardware.Intake;
import org.firstinspires.ftc.teamcode.Voltrons.hardware.Launcher;
import org.firstinspires.ftc.teamcode.Voltrons.hardware.WoobleArm;

@Config
@TeleOp (name = "Driver", group = "Driver")
public class VoltronsDriver extends LinearOpMode {


    public static double wobbleHandMin = 0;
    public static double wobbleHandMax = 180; // Degrees
    public static double wobbleArmMin = 0;
    public static double wobbleArmMax = 270;

    public static double kp;
    public static double kd;
    public static double ki;
    public static double kf;


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


        // Gamepads
        GamepadEx gpad1 = new GamepadEx(gamepad1);
        GamepadEx gpad2 = new GamepadEx(gamepad2);

        // Button readers
        ButtonReader aReader1 = new ButtonReader(
                gpad1, GamepadKeys.Button.A
        );

        ButtonReader yReader1 = new ButtonReader(
                gpad1, GamepadKeys.Button.Y
        );

        ButtonReader aReader2 = new ButtonReader(
                gpad2, GamepadKeys.Button.A
        );

        // Hardware
        PIDFController pidf = new PIDFController(kp,kd,ki,kf);
        MecanumDrive mecanum = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        Belt belt = new Belt(beltDown, betlUp);
        Intake intake = new Intake(intakeMotor);
        Launcher launcher = new Launcher(leftLauncher, rightLauncher);
        WoobleArm woobleArm = new WoobleArm(wobbleArm, wobbleHand, pidf, wobbleArmMin, wobbleArmMax, 1000);

        // Util
        boolean slowMode = false;
        int invert = 1;
        double goalPosition = 0;

        waitForStart();


        while(opModeIsActive())
        {

            // Drivetrain
            if (slowMode) {
                mecanum.driveRobotCentric(
                        invert * gpad1.getLeftX() * 0.3,
                        invert * gpad1.getLeftY() * 0.3,
                        invert * gpad1.getRightX() * 0.3
                );
            }
            else
            {
                mecanum.driveRobotCentric(
                        invert * gpad1.getLeftX(),
                        invert * gpad1.getLeftY(),
                        invert * gpad1.getRightX()
                );
            }

            // Slow Mode
            aReader1.readValue();
            if (aReader1.wasJustPressed())
            {
                slowMode = !slowMode;
            }

            // Invert mode
            yReader1.readValue();
            if (yReader1.wasJustPressed())
            {
                if (invert == 1) {
                    invert = -1;
                }
                else {
                    invert = 1;
                }
            }

            // Intake
            double leftTriggerPower1 = gpad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
            double rightTriggerPower1 = gpad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

            if (leftTriggerPower1 > 0) {
                intake.setPower(-leftTriggerPower1);
            }
            else if (rightTriggerPower1 > 0) {
                intake.setPower(rightTriggerPower1);
            }
            else {
                intake.setPower(0);
            }

            // Belt
            if (gpad1.getButton(GamepadKeys.Button.LEFT_BUMPER) || gpad2.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                belt.moveDown();
            }
            else if (gpad1.getButton(GamepadKeys.Button.RIGHT_BUMPER) || gpad2.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                belt.moveUp();
            }
            else {
                belt.stop();
            }

            // Launcher
            double leftTriggerPower2 = gpad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
            double rightTriggerPower2 = gpad2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

            if (leftTriggerPower2 > 0) {
                launcher.setPower(-leftTriggerPower2);
            }
            else if (rightTriggerPower2 > 0) {
                launcher.setPower(rightTriggerPower2);
            }
            else {
                launcher.setPower(0);
            }

            // Arm
            aReader2.readValue();
            if (aReader2.wasJustPressed()) {
                if (woobleArm.isOpen()) {
                    woobleArm.closeHand();
                }
                else {
                    woobleArm.openHand();
                }
            }




        }
    }
}
