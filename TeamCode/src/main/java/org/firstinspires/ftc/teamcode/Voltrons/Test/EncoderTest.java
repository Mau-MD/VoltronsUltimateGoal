package org.firstinspires.ftc.teamcode.Voltrons.Test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@Autonomous(name="Encoder Test", group="Test")
public class EncoderTest extends LinearOpMode {

    DcMotor left_front;
    DcMotor right_front;
    DcMotor left_back;
    DcMotor right_back;

    DcMotor wooble_arm;
    Servo wooble_hand;

    DcMotor intake;
    CRServo belt_down;
    CRServo belt_up;
    DcMotor left_launcher;
    DcMotor right_launcher;

    public static double k_p = 0.003;
    public static int distance = 5000;
    ElapsedTime aButton = new ElapsedTime();
    ElapsedTime bButton = new ElapsedTime();
    ElapsedTime yButton = new ElapsedTime();
    ElapsedTime a2Button = new ElapsedTime();
    ElapsedTime woobleDelay = new ElapsedTime();

    BNO055IMU imu;

    @Override
    public void runOpMode() {

        left_front = hardwareMap.dcMotor.get("lf");
        right_front = hardwareMap.dcMotor.get("rf");
        left_back = hardwareMap.dcMotor.get("lb");
        right_back = hardwareMap.dcMotor.get("rb");

        wooble_arm = hardwareMap.dcMotor.get("wa");
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

        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_front.setDirection(DcMotorSimple.Direction.REVERSE);

        aButton.reset();
        bButton.reset();
        yButton.reset();
        a2Button.reset();
        woobleDelay.reset();

        // PID
        double position_goal = 0;

        // Encoder

        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_back.setTargetPosition(1000);

        right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        waitForStart();
        if (opModeIsActive()) {

            right_back.setPower(0.4);

            while (right_back.isBusy()) {
                double encoder_power = right_back.getPower();
                telemetry.addData("Power", encoder_power);
                telemetry.update();
                left_back.setPower(encoder_power);
                right_front.setPower(encoder_power);
                left_front.setPower(encoder_power);
            }

            left_back.setPower(0);
            right_back.setPower(0);
            left_front.setPower(0);
            right_front.setPower(0);

            sleep(5000);
        }
    }
}
