package org.firstinspires.ftc.teamcode.Voltrons.hardware;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Drivetrain {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private Imu imu;

    public Drivetrain(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, BNO055IMU imu) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.imu = new Imu(imu);
    }

    public void idle() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void driveGyro(double[] power, double angle, double time, PIDFController pidf) {
        ElapsedTime currentTime = new ElapsedTime();
        currentTime.reset();

        double correction = 0;
        while (currentTime.milliseconds() < time) {

            frontLeft.setPower(power[0] + correction);
            frontRight.setPower(power[1] - correction);
            backLeft.setPower(power[2] + correction);
            backRight.setPower(power[3] - correction);

            double error = Imu.getError(imu.getAngleNormalized(), angle);
            correction = pidf.calculate(error,angle); // Mal
        }

        idle();
    }

    public void setOrientation(double power, double angle, PIDController pidf) {

        while (true) {

            double error = Imu.getError(imu.getAngleNormalized(), angle);
            double correction = pidf.calculate(error, angle);
            pidf.calculate(error)

            frontLeft.setPower(-power * correction);
            frontRight.setPower(power * correction);
            backLeft.setPower(-power * correction);
            backRight.setPower(power * correction);

            if (Math.abs(error) < 1.0) break;
        }
    }

}
