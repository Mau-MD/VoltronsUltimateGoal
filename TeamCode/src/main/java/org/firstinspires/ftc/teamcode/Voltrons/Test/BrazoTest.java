package org.firstinspires.ftc.teamcode.Voltrons.Test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Config
@TeleOp(name="Prueba Wobble", group="Test")
public class BrazoTest extends LinearOpMode {

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
    public void runOpMode() {

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

        waitForStart();

        double codoPos = 0;
        double manoPos = 0;

        int positionGoal = -130;

        while (opModeIsActive()) {

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

            if (gamepad1.a && botonA.milliseconds() > 300)
            {
                positionGoal += 100;
                botonA.reset();
            }

            if (gamepad1.b && botonB.milliseconds() > 300)
            {
                positionGoal -= 100;
                botonB.reset();
            }

            if (gamepad1.x && botonB.milliseconds() > 300)
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