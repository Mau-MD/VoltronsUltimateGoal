package org.firstinspires.ftc.teamcode.OpMode;

import android.widget.Button;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name = "Driver", group = "Driver")
public class VoltronsDriver extends LinearOpMode {

    double intakePower;
    DcMotor brazo;
    Servo codo;
    Servo mano;

    ElapsedTime dpadArriba = new ElapsedTime();
    ElapsedTime dpadAbajo = new ElapsedTime();
    ElapsedTime dpadIzquierda = new ElapsedTime();
    ElapsedTime dpadDerecha = new ElapsedTime();
    ElapsedTime botonA = new ElapsedTime();
    ElapsedTime botonB = new ElapsedTime();

    public static double proportional = 0.003;
    public static double integral = 0;
    public static double derivative = 0;


    @Override
    public void runOpMode()
    {
        // Drivetrain
        Motor frontLeft = new Motor(hardwareMap, "fl", Motor.GoBILDA.RPM_117);
        Motor frontRight = new Motor(hardwareMap, "fr", Motor.GoBILDA.RPM_117);
        Motor backLeft = new Motor(hardwareMap, "bl", Motor.GoBILDA.RPM_117);
        Motor backRight= new Motor(hardwareMap, "br", Motor.GoBILDA.RPM_117);

        Motor intake = new Motor(hardwareMap, "in", Motor.GoBILDA.RPM_312);

        frontLeft.setRunMode(Motor.RunMode.RawPower);
        frontRight.setRunMode(Motor.RunMode.RawPower);
        backLeft.setRunMode(Motor.RunMode.RawPower);
        backRight.setRunMode(Motor.RunMode.RawPower);

        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // Quita los "//" a los motores que esten invertidos

        // frontLeft.setInverted(1);
        // frontRight.setInverted(1);
        // backLeft.setInverted(1);
        // backRight.setInverted(1);

        MecanumDrive mecanum = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        // Gamepads
        GamepadEx gpad1 = new GamepadEx(gamepad1);
        GamepadEx gpad2 = new GamepadEx(gamepad2);

        // Button readers
        ButtonReader aReader = new ButtonReader(
                gpad1, GamepadKeys.Button.A
        );

        dpadArriba.reset();
        dpadAbajo.reset();
        dpadIzquierda.reset();
        dpadDerecha.reset();
        botonA.reset();
        botonB.reset();

        brazo = hardwareMap.dcMotor.get("br");
        codo = hardwareMap.servo.get("cd");
        mano = hardwareMap.servo.get("mn");

        brazo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        boolean slowMode = false;

        waitForStart();

        double codoPos = 0;
        double manoPos = 0;

        int positionGoal = -130;

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

            // Intake Power
            intakePower = 0;

            if (gpad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0)
            {
                intakePower = gpad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
            }
            else if (gpad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0)
            {
                intakePower = -gpad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
            }

            intake.set(intakePower);

            // Brazo

            double brazoPower = 0.0;

            if (gamepad1.right_trigger > 0.0 && dpadArriba.milliseconds() > 10)
            {
                positionGoal += (gamepad1.right_trigger * 10);
                dpadArriba.reset();
            }
            else if (gamepad1.left_trigger > 0.0 && dpadArriba.milliseconds() > 10)
            {
                positionGoal -= (gamepad1.left_trigger * 10);
                dpadArriba.reset();
            }

            if (gamepad1.x && botonA.milliseconds() > 300)
            {
                positionGoal += 100;
                botonA.reset();
            }

            if (gamepad1.b && botonB.milliseconds() > 300)
            {
                positionGoal -= 100;
                botonB.reset();
            }

            if (gamepad1.y && botonB.milliseconds() > 300)
            {
                positionGoal = -130;
                botonB.reset();
            }

            if (gamepad1.dpad_up && dpadArriba.milliseconds() > 300) {
                codoPos += 0.1;
                dpadArriba.reset();
            }
            else if (gamepad1.dpad_down && dpadAbajo.milliseconds() > 300)
            {
                codoPos -= 0.1;
                dpadAbajo.reset();
            }

            if (gamepad1.dpad_right && dpadDerecha.milliseconds() > 300)
            {
                manoPos += 0.1;
                dpadDerecha.reset();
            }
            else if (gamepad1.dpad_left && dpadIzquierda.milliseconds() > 300)
            {
                manoPos -= 0.1;
                dpadIzquierda.reset();
            }


            double error = positionGoal - brazo.getCurrentPosition();
            brazoPower = error * proportional;

            codoPos = Range.clip(codoPos,0.0,1.0);
            manoPos = Range.clip(manoPos, 0.0, 1.0);
            brazoPower = Range.clip(brazoPower, -1.0, 1.0);


            telemetry.addData("Codo Pos: ", codoPos);
            telemetry.addData("Mano Pos: ", manoPos);
            telemetry.addData("Brazo Power: ", brazoPower);
            telemetry.addData("PositionGoal: ", positionGoal);
            telemetry.addData("Error: ", error);
            telemetry.update();

            codo.setPosition(codoPos);
            mano.setPosition(manoPos);
            brazo.setPower(brazoPower);

        }
    }
}
