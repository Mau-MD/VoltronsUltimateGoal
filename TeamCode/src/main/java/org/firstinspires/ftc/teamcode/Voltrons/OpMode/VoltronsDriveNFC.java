package org.firstinspires.ftc.teamcode.Voltrons.OpMode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.List;


@Config
@TeleOp(name="Driver NFC", group="UO")
public class VoltronsDriveNFC extends LinearOpMode {

    DcMotor left_front;
    DcMotor right_front;
    DcMotor left_back;
    DcMotor right_back;

    Motor wooble_arm;
    Servo wooble_hand;

    DcMotor intake;
    CRServo belt_down;
    CRServo belt_up;
    DcMotor left_launcher;
    DcMotor right_launcher;

    // Config Variables via Dashboard

    public static int open = 500;
    public static int closed = 0;

    public static double k_p = 0.003;

    ElapsedTime aButton = new ElapsedTime();
    ElapsedTime bButton = new ElapsedTime();
    ElapsedTime yButton = new ElapsedTime();
    ElapsedTime a2Button = new ElapsedTime();
    ElapsedTime woobleDelay = new ElapsedTime();

    //File UpPath = AppUtil.getInstance().getSettingsFile("UpMovement.txt");
    //File DownPath = AppUtil.getInstance().getSettingsFile("DownMovement.txt");


    @Override
    public void runOpMode() {

        left_front = hardwareMap.dcMotor.get("fl");
        right_front = hardwareMap.dcMotor.get("fr");
        left_back = hardwareMap.dcMotor.get("bl");
        right_back = hardwareMap.dcMotor.get("br");

        wooble_arm = new Motor(hardwareMap, "wa");
        wooble_hand = hardwareMap.servo.get("wh");

        intake = hardwareMap.dcMotor.get("in");
        belt_down = hardwareMap.crservo.get("bd");
        belt_up = hardwareMap.crservo.get("bu");

        // 1
        right_launcher = hardwareMap.dcMotor.get("rl");
        // 0
        left_launcher = hardwareMap.dcMotor.get("ll");

        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wooble_arm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_front.setDirection(DcMotorSimple.Direction.REVERSE);

        wooble_arm.setInverted(true);
        wooble_arm.resetEncoder();
        wooble_arm.resetEncoder();
        wooble_arm.resetEncoder();

        aButton.reset();
        bButton.reset();
        yButton.reset();
        a2Button.reset();
        woobleDelay.reset();

        double adjust = 10;
        boolean intake_gamepad1 = false;
        boolean hand_open = false;
        int invert = 1;

        // PID
        double position_goal = wooble_arm.getCurrentPosition();
        hand_open = true;

        ElapsedTime downTime = new ElapsedTime();
        ElapsedTime upTime = new ElapsedTime();
        List<Double>down = new ArrayList<>();
        List<Double>up = new ArrayList<>();
        boolean upRecording = false;
        boolean downRecording = false;
        double finalDownTime = 0;
        double finalUpTime = 0;

        waitForStart();
        while (opModeIsActive()) {

            // invertMotors();
            right_front.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + (gamepad1.right_stick_x * invert)) * (adjust/10.0));
            left_front.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - (gamepad1.right_stick_x * invert)) * (adjust/10.0));
            right_back.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + (gamepad1.right_stick_x * invert)) * (adjust/10.0));
            left_back.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - (gamepad1.right_stick_x * invert)) * (adjust/10.0));

            // Slow Mode
            if (gamepad1.a && aButton.milliseconds() > 300)
            {
                if (adjust == 10)
                {
                    adjust = 4;
                }
                else
                {
                    adjust = 10;
                }
                aButton.reset();
            }

            // Invert Mode
            if (gamepad1.y && yButton.milliseconds() > 300)
            {
                if (invert == 1)
                {
                    left_front.setDirection(DcMotorSimple.Direction.FORWARD);
                    right_front.setDirection(DcMotorSimple.Direction.FORWARD);
                    left_back.setDirection(DcMotorSimple.Direction.REVERSE);
                    right_back.setDirection(DcMotorSimple.Direction.REVERSE);
                    invert = -1;
                }
                else {
                    left_front.setDirection(DcMotorSimple.Direction.REVERSE);
                    right_front.setDirection(DcMotorSimple.Direction.REVERSE);
                    left_back.setDirection(DcMotorSimple.Direction.FORWARD);
                    right_back.setDirection(DcMotorSimple.Direction.FORWARD);
                    invert = 1;
                }
                yButton.reset();
            }

            // Intake First Gamepad

            if (gamepad1.dpad_right)
            {
                intake_gamepad1 = true;
                intake.setPower(-1);
            }
            else if (gamepad1.dpad_left)
            {
                intake_gamepad1 = true;
                intake.setPower(1);
            }
            else
            {
                intake_gamepad1 = false;
                intake.setPower(0);
            }

            // Belt
            if (gamepad2.right_bumper)
            {
                belt_up.setPower(1);
                belt_down.setPower(1);
            }
            else if (gamepad2.left_bumper)
            {
                belt_up.setPower(-1);
                belt_down.setPower(-1);
            }
            else
            {
                belt_up.setPower(0);
                belt_down.setPower(0);
            }
            // Launcher Gamepead 1

            left_launcher.setPower(-gamepad1.right_trigger);
            right_launcher.setPower(gamepad1.right_trigger);

            left_launcher.setPower(gamepad1.left_trigger);
            right_launcher.setPower(-gamepad1.left_trigger);

            // Launcher Gamepad 2

            left_launcher.setPower(-gamepad2.right_trigger);
            right_launcher.setPower(gamepad2.right_trigger);

            left_launcher.setPower(gamepad1.left_trigger);
            right_launcher.setPower(-gamepad1.left_trigger);

            // Wooble
            if (gamepad2.a && a2Button.milliseconds() > 300)
            {
                if (hand_open)
                {
                    wooble_hand.setPosition(0);
                    hand_open = false;
                }
                else
                {
                    wooble_hand.setPosition(1);
                    hand_open = true;
                } ;
                a2Button.reset();
            }





            if (Math.abs(gamepad2.left_stick_y) > 0 && woobleDelay.milliseconds() > 20)
            {
                position_goal += -gamepad2.left_stick_y * 20;
                woobleDelay.reset();
            }
            if (Math.abs(gamepad2.right_stick_y) > 0 && woobleDelay.milliseconds() > 20)
            {
                position_goal += -gamepad2.right_stick_y * 10;
                woobleDelay.reset();
            }

            double error = position_goal - wooble_arm.getCurrentPosition();
            double wooble_arm_power = k_p * error;


            wooble_arm.set(wooble_arm_power);
            telemetry.addData("power", wooble_arm_power);
            telemetry.addData("position goal", position_goal);
            telemetry.addData("current position", wooble_arm.getCurrentPosition());
            telemetry.update();

        }
    }
}
