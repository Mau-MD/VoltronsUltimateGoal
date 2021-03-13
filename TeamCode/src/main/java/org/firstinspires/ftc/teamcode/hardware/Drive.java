package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.Voltrons.Vision.functions.UtilFunctions.*;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Drive {

    /**
     * Direcciones utilizadas para especificar a donde se quiere conducir en las funciones
     * driveTimed(), driveTimedGyro(), etc.
     * Note: en las funciones de turnTimed() y turnGyro() solo recibe las directions LEFT y RIGHT.
     */
    public enum direction {
        FORWARD, BACKWARDS, LEFT, RIGHT;
    }

    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private boolean invertedMotors[] = new boolean[4];

    private BNO055IMU imu;

    private double proportional;
    private double turnProportional;
    private double derivative;
    private double integral;

    /**
     * Inicializa la clase. Recibe como entrada los 4 motores
     * @param leftFront
     * @param rightFront
     * @param leftBack
     * @param rightBack
     */
    public Drive(DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack)
    {
        this.leftFront = leftFront;
        this.rightBack = rightBack;
        this.leftBack  = leftBack;
        this.rightFront = rightFront;

        for (int i = 0; i < 4; i ++)invertedMotors[i] = false;
    }

    /**
     * Inicializa la clase. Recibe como entrada los 4 motores mas el IMU. Si se inicializa de esta manera
     * se podran utilizar clases como driveTimedGyro() o turnGyro().
     * @param leftFront
     * @param rightFront
     * @param leftBack
     * @param rightBack
     * @param imu
     */
    public Drive(DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack, BNO055IMU imu)
    {
        this.leftFront = leftFront;
        this.rightBack = rightBack;
        this.leftBack  = leftBack;
        this.rightFront = rightFront;

        for (int i = 0; i < 4; i ++)invertedMotors[i] = false;

        this.imu = imu;
    }


    // Motor functions

    /**
     * Pone todos los motores en modo freno
     */
    public void setBrake()
    {
        setBrake(leftFront);
        setBrake(rightFront);
        setBrake(leftBack);
        setBrake(rightBack);
    }

    /**
     * Pone el motor dado en modo freno
     * @param motor
     */
    public void setBrake(DcMotor motor)
    {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Invierte todos los motores
     * @param direction un array de booleanos donde 1 significa el motor que se quiere invertir y 0 el que no
     */
    public void invertMotors(boolean [] direction)
    {
        if (direction[0])
        {
            invertMotor(leftFront, DcMotorSimple.Direction.REVERSE);
        }
        else
        {
            invertMotor(leftFront, DcMotorSimple.Direction.FORWARD);
        }

        if (direction[1])
        {
            invertMotor(rightFront, DcMotorSimple.Direction.REVERSE);
        }
        else
        {
            invertMotor(rightFront, DcMotorSimple.Direction.FORWARD);
        }
        if (direction[2])
        {
            invertMotor(leftBack, DcMotorSimple.Direction.REVERSE);
        }
        else
        {
            invertMotor(leftBack, DcMotorSimple.Direction.FORWARD);
        }
        if (direction[3])
        {
            invertMotor(rightBack, DcMotorSimple.Direction.REVERSE);
        }
        else
        {
            invertMotor(rightBack, DcMotorSimple.Direction.FORWARD);
        }
    }

    /**
     * Inviterte un solo motor especificado
     * @param motor
     * @param direction Direccion la cual se quiere invterir (E.g. DcMotorSimple.Direction.FORWARD)
     */
    public void invertMotor(DcMotor motor, DcMotorSimple.Direction direction)
    {
        motor.setDirection(direction);
    }

    // PID Functions

    public void setProportional(double proportional)
    {
        this.proportional = proportional;
    }

    public void setDerivative(double derivative)
    {
        this.derivative = derivative;
    }

    public void setIntegral(double integral)
    {
        this.integral = integral;
    }

    /**
     * Proportional especial para los giros
     * @param turnProportional
     */
    public void setTurnProportional(double turnProportional)
    {
        this.turnProportional = turnProportional;
    }

    /**
     * Ingresas todas las constantes para PID
     * @param proportional
     * @param derivative
     * @param integral
     */
    public void setPIDCoeff(double proportional, double derivative, double integral)
    {
        this.proportional = proportional;
        this.derivative = derivative;
        this.integral = integral;
    }

    // Drive Functions

    /**
     * Recibe un poder especifico para cada motor
     * @param leftFront
     * @param rightFront
     * @param leftBack
     * @param rightBack
     */
    public void drive(double leftFront, double rightFront, double leftBack, double rightBack)
    {
        this.leftFront.setPower(leftFront);
        this.rightFront.setPower(rightFront);
        this.leftBack.setPower(leftBack);
        this.rightBack.setPower(rightBack);
    }

    /**
     * Maneja el robot de acuerdo a una direccion y un tiempo expresado en milisegundos
     * @param dir Direccion la cual quiere que se maneje (E.g. Drive.Direction.FORWARD)
     * @param power Potencia de los motores
     * @param miliseconds Tiempo
     */
    public void driveTimed(direction dir, double power, int miliseconds)
    {
        power = Math.abs(power);

        if (dir == direction.FORWARD)
        {
            drive(power,power,power,power);
            sleep(miliseconds);
            drive(0,0,0,0);
        }
        else if (dir == direction.BACKWARDS)
        {
            drive(-power,-power,-power,-power);
            sleep(miliseconds);
            drive(0,0,0,0);
        }
        else if (dir == direction.LEFT)
        {
            drive(power,-power,power,-power); // Need revision
            sleep(miliseconds);
            drive(0,0,0,0);
        }
        else if (dir == direction.RIGHT)
        {
            drive(-power,power,-power,power); // Need revision
            sleep(miliseconds);
            drive(0,0,0,0);
        }
    }

    /**
     * Giro del robot por un timepo determinado
     * @param dir Direccion la cual quiere que se maneje (E.g. Drive.Direction.LEFT)
     * @param power Potencia de los motores
     * @param miliseconds Tiempo que se quiere ejecutar
     */
    public void turnTimed(direction dir, double power, int miliseconds)
    {
        if (dir == direction.RIGHT)
        {
            drive(power,-power,-power,power); // Need revision
            sleep(miliseconds);
            drive(0,0,0,0);
        }
        else if (dir == direction.LEFT)
        {
            drive(-power,power,power,-power); // Need revision
            sleep(miliseconds);
            drive(0,0,0,0);
        }
    }

    // Drive with gyro

    /**
     * Maneja el robot de acuerdo a un tiempo determinado. Se corrige automaticamente de acuerdo a un angulo para siempre
     * mantenerlo y no desviarse debido a la friccion o algun choque
     * @param dir Direccion la cual quiere que se maneje (E.g. Drive.Direction.RIGHT)
     * @param power Potencia de los motores
     * @param miliseconds Tiempo
     * @param angle Angulo en RADIANES
     */
    public void driveTimedGyro(direction dir, double power, int miliseconds, double angle)
    {
        power = Math.abs(power);

        ElapsedTime time = new ElapsedTime();
        time.reset();

        if (dir == direction.FORWARD)
        {
            while (time.milliseconds() < miliseconds)
            {
                double heading = AngleWrap(getHeading(imu));
                double error = getError(angle, heading);

                double steering = error * proportional;

                drive(power + steering, power - steering, power + steering, power - steering);
            }
            drive(0,0,0,0);
        }
        else if (dir == direction.BACKWARDS)
        {
            while (time.milliseconds() < miliseconds)
            {
                double heading = AngleWrap(getHeading(imu));
                double error = getError(angle, heading);

                double steering = error * proportional;

                drive(-power + steering, -power - steering, -power + steering, -power - steering);
            }
            drive(0,0,0,0);
        }
        else if (dir == direction.LEFT)
        {
            while (time.milliseconds() < miliseconds)
            {
                double heading = AngleWrap(getHeading(imu));
                double error = getError(angle, heading);

                double steering = error * proportional;

                drive(-power + steering, power - steering, power + steering, -power - steering);
            }
            drive(0,0,0,0);
        }
        else if (dir == direction.RIGHT)
        {
            while (time.milliseconds() < miliseconds)
            {
                double heading = AngleWrap(getHeading(imu));
                double error = getError(angle, heading);

                double steering = error * proportional;

                drive(power + steering, -power - steering, -power + steering, power - steering);
            }
            drive(0,0,0,0);
        }
    }

    /**
     * Giro preciso mediante el uso del giroscopio. No se dentendra el codigo hasta llegar al angulo objetivo
     * @param dir Direccion la cual quiere que se maneje (E.g. Drive.Direction.LEFT)
     * @param power Poder para girar
     * @param angle Angulo al cual se quiere llegar
     * Note: Es recomendable dejar el poder en 1. Varia dependiendo de los motores.
     */
    public void turnGyro(direction dir, double power, double angle)
    {
        double error = 999;

        if (dir == direction.RIGHT) {
            while (error > 3) {
                double heading = AngleWrap(getHeading(imu));
                error = getError(angle, heading);

                double motorPower = error * turnProportional;
                drive(motorPower * power, -motorPower * power, motorPower * power, -motorPower * power);
            }
            drive(0, 0, 0, 0);
        }
        else if (dir == direction.LEFT)
        {
            while (error > 3) {
                double heading = AngleWrap(getHeading(imu));
                error = getError(angle, heading);

                double motorPower = error * turnProportional;
                drive(-motorPower * power, motorPower * power, -motorPower * power, motorPower * power);
            }
            drive(0, 0, 0, 0);
        }
    }
}
