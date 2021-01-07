package org.firstinspires.ftc.teamcode.OpMode;

import android.widget.Button;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "Driver", group = "Driver")
public class VoltronsDriver extends LinearOpMode {


    @Override
    public void runOpMode()
    {
        // Drivetrain
        Motor frontLeft = new Motor(hardwareMap, "fl", Motor.GoBILDA.RPM_117);
        Motor frontRight = new Motor(hardwareMap, "fr", Motor.GoBILDA.RPM_117);
        Motor backLeft = new Motor(hardwareMap, "bl", Motor.GoBILDA.RPM_117);
        Motor backRight= new Motor(hardwareMap, "br", Motor.GoBILDA.RPM_117);

        frontLeft.setRunMode(Motor.RunMode.RawPower);
        frontRight.setRunMode(Motor.RunMode.RawPower);
        backLeft.setRunMode(Motor.RunMode.RawPower);
        backRight.setRunMode(Motor.RunMode.RawPower);

        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        MecanumDrive mecanum = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        // Gamepads
        GamepadEx gpad1 = new GamepadEx(gamepad1);
        GamepadEx gpad2 = new GamepadEx(gamepad2);

        // Button readers
        ButtonReader aReader = new ButtonReader(
                gpad1, GamepadKeys.Button.A
        );

        boolean slowMode = false;

        waitForStart();


        while(opModeIsActive())
        {

            if (slowMode) {
                mecanum.driveRobotCentric(0.5, 0.5, 0.5);
            }
            else
            {
                mecanum.driveRobotCentric(1, 1, 1);
            }

            aReader.readValue();
            if (aReader.wasJustPressed())
            {
                slowMode = !slowMode;
            }

        }
    }
}
