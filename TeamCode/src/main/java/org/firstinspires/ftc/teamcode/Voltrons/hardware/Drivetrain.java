package org.firstinspires.ftc.teamcode.Voltrons.hardware;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Voltrons.Constants;

public class Drivetrain {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private Imu imu;

    private PIDFController orientationPIDF;
    private PIDFController gyroPIDF;
    private PIDFController encoderPIDF;

    /**
     * Initializes drivetrain class. It requires 4 dc motors and a IMU object
     * @param frontLeft frontLeft motor
     * @param frontRight frontRight motor
     * @param backLeft backLeft motor
     * @param backRight backRight motor
     * @param imu {@link BNO055IMU} IMU
     */
    public Drivetrain(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, BNO055IMU imu) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.imu = new Imu(imu);
    }

    /**
     * Stops all the 4 drivetrain motors
     */
    public void idle() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    /**
     * Sets the gyroPIDF to some specific coeffs
     * @param gyroPIDF
     */
    public void setGyroPIDF(PIDFController gyroPIDF) {
        this.gyroPIDF = gyroPIDF;
    }

    /**
     * Drives the robot following an specific angle
     * @param power motor's power
     * @param angle desired angle to follow
     * @param time drive time
     * @param pidf pidf controller
     */
    public void driveGyro(double[] power, double angle, double time, PIDFController pidf) {
        setGyroPIDF(pidf);
        driveGyro(power,angle,time);
    }

    /**
     * Drives the robot following an specific angle. It requires to have set a pidf controller before with setGyroPIDF()
     * @param power motor's power
     * @param angle desired angle to follow
     * @param time drive time
     */
    public void driveGyro(double[] power, double angle, double time) {
        ElapsedTime currentTime = new ElapsedTime();
        currentTime.reset();

        double correction = 0;
        while (currentTime.milliseconds() < time) {

            frontLeft.setPower(power[0] + correction);
            frontRight.setPower(power[1] - correction);
            backLeft.setPower(power[2] + correction);
            backRight.setPower(power[3] - correction);

            double error = Imu.getError(imu.getAngleNormalized(), angle);
            correction = gyroPIDF.calculate(error,0);
        }

        idle();
    }

    /**
     * Sets the encoderPIDF to some specific coeffs
     * @param encoderPIDF
     */
    public void setEncoderPIDF(PIDFController encoderPIDF) {
        this.encoderPIDF = encoderPIDF;
    }

    /**
     * Drives the robot following an specific angle traveling an specified amount of centimeters
     * @param power motor's power
     * @param angle angle to follow
     * @param goal cm to move
     * @param gyroPIDF gyroPIDF controller
     * @param encoderPIDF encoderPIDF controller
     */
    public void driveEncoderGyro(double[] power, double angle, double goal, PIDFController gyroPIDF, PIDFController encoderPIDF) {
        setGyroPIDF(gyroPIDF);
        setEncoderPIDF(encoderPIDF);
        driveEncoderGyro(power, angle, goal);
    }

    /**
     * Drives the robot following an specific angle traveling an specified amount of centimeters it requires to have set
     * a gyro PIDF controller and a encoder PIDF controller
     * @param power motor's power
     * @param angle angle to follow
     * @param goal cm to move
     */
    public void driveEncoderGyro(double[] power, double angle, double goal) {
        // Quiero avanzar x cm desde donde sea que este
        double relativePosition = 0;
        double startingPosition = (backLeft.getCurrentPosition() + backRight.getCurrentPosition()) / 2.0;
        double ticksGoal = Drivetrain.cmToTicks(goal);

        // Relative Ticks
        while (true) {

            relativePosition = ((backLeft.getCurrentPosition() + backRight.getCurrentPosition()) / 2.0) - startingPosition;
            double encoderOutput = encoderPIDF.calculate(relativePosition, ticksGoal);
            double angleError = Imu.getError(imu.getAngleNormalized(), angle);
            double angleCorrection = orientationPIDF.calculate(angleError, 0);

            frontLeft.setPower(encoderOutput * power[0] + angleCorrection);
            frontRight.setPower(encoderOutput * power[1] - angleCorrection);
            backLeft.setPower(encoderOutput * power[2] + angleCorrection);
            backRight.setPower(encoderOutput * power[3] - angleCorrection);

            if (Math.abs(encoderOutput) < 0.2)break;
        }
        idle();
    }

    /**
     * Sets the gyroPIDF to some specific coeffs
     * @param orientationPIDF pidf controller
     */
    public void setOrientationPIDF(PIDController orientationPIDF) {
        this.orientationPIDF = orientationPIDF;
    }

    /**
     * Turns the robot to an specific angle
     * @param power motor's power
     * @param angle angle to turn
     * @param pidf pidf controller
     */
    public void setOrientation(double power, double angle, PIDController pidf) {
        setOrientationPIDF(pidf);
        setOrientation(power,angle);
    }

    /**
     * Turns the robot to an specific angle. It requires to have set a orientationPIDF controller.
     * @param power
     * @param angle
     */
    public void setOrientation(double power, double angle) {

        while (true) {

            double error = Imu.getError(imu.getAngleNormalized(), angle);
            double correction =  orientationPIDF.calculate(error,0);

            frontLeft.setPower(-power * correction);
            frontRight.setPower(power * correction);
            backLeft.setPower(-power * correction);
            backRight.setPower(power * correction);

            if (Math.abs(error) < 1.0) break;
        }

        idle();
    }

    /**
     * Converts cm to motor ticks
     * @param cm cm
     * @return motor ticks
     */
    public static double cmToTicks(double cm) {
        return cm * Constants.TICKS_PER_REVOLUTION / Constants.WHEEl_DIAMETER_CM;
    }

    /**
     * Converts motor ticks to cm
     * @param ticks ticks
     * @return motor ticks
     */
    public static double ticksToCm(double ticks) {
        return ticks * Constants.WHEEl_DIAMETER_CM / Constants.TICKS_PER_REVOLUTION;
    }

}
