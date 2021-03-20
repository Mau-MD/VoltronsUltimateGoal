package org.firstinspires.ftc.teamcode.Voltrons.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Imu {

    private BNO055IMU imu;


    public Imu(BNO055IMU imu) {
        this.imu = imu;
    }

    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public double getAngleNormalized() {
        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        if (angle <= 0) {
            return 180 + Math.abs(angle);
        }
        else {
            return 180 - angle;
        }
    }

    // Error negativo significa ir a la izquierda. Positivo derecha
    public static double getError(double angle, double goal) {
        double error = goal - angle;
        while (error > 180) {
            error -= 360;
        }
        while (error < -180) {
           error += 360;
        }
        return error;
    }

    public static double normalizeSplineAngle(double angle) {
        double newAngle = angle;
        newAngle %= 360;
        return newAngle;
    }

}
