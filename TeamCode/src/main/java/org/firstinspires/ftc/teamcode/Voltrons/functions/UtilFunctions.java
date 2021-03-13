package org.firstinspires.ftc.teamcode.Voltrons.Vision.functions;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class UtilFunctions {

    /**
     * Pausa el programa por un determinado tiempo en milisegundos
     * @param miliseconds
     */
    public static void sleep(int miliseconds)
    {
        ElapsedTime time = new ElapsedTime();
        time.reset();
        while(time.milliseconds() < miliseconds)
        {
            // Nothing
        }
    }

    /**
     * Normaliza el angulo dado para que este en el rango -180 a 180
     * @param angle
     * @return angulo normalizado
     */
    public static double AngleWrap(double angle)
    {
        while (angle < -Math.PI)
        {
            angle += 2 * Math.PI;
        }
        while (angle > Math.PI)
        {
            angle -= 2 * Math.PI;
        }
        return angle;
    }

    /**
     * Regresa el error entre un objetivo y un valor actual
     * @param obj objetivo
     * @param actual
     * @return el error
     */
    public static double getError(double obj, double actual)
    {
        return obj - actual;
    }

    /**
     * Obtiene el heading en RADIANES del IMU
     * @param imu la variable del IMU
     * @return heading en RADIANES
     */
    public static double getHeading(BNO055IMU imu)
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return angles.firstAngle;
    }

}
