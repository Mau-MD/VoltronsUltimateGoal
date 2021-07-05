package org.firstinspires.ftc.teamcode.Voltrons.OpMode.Autonomous.Test;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "ArmTest", group = "test")
public class ArmTest extends LinearOpMode {

    Motor arm;
    Servo hand;

    public static int downPosition = 500;
    public static int upPosition = 0;
    public static double power = 0.5;


    @Override
    public void runOpMode() {

        hand = hardwareMap.servo.get("hand");
        arm = new Motor(hardwareMap, "arm", Motor.GoBILDA.RPM_117);

        int target = 0;
        boolean open = false;
        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.b) {
               if (open) {
                   hand.setPosition(1);
                   open = false;
               }
               else {
                   hand.setPosition(0);
                   open = true;
               }
            }
            if (gamepad1.a) {
                target = downPosition;
            }
            if (gamepad1.y) {
                target = upPosition;
            }

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
}
