package org.firstinspires.ftc.teamcode.Voltrons.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.drive.Drive;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Voltrons.Constants;
import org.firstinspires.ftc.teamcode.Voltrons.Path.Spline;
import org.firstinspires.ftc.teamcode.Voltrons.control.PID;

public class Drivetrain {

    private Motor frontLeft;
    private Motor frontRight;
    private Motor backLeft;
    private Motor backRight;
    private Imu imu;

    private PID encoderPID;
    private PID gyroPID;
    private PID orientationPID;

    private FtcDashboard dashboard;
    private TelemetryPacket packet;

    /**
     * Initializes drivetrain class. It requires 4 dc motors and a IMU object
     * @param frontLeft frontLeft motor
     * @param frontRight frontRight motor
     * @param backLeft backLeft motor
     * @param backRight backRight motor
     * @param imu {@link BNO055IMU} IMU
     */
    public Drivetrain(Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight, BNO055IMU imu) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.imu = new Imu(imu);
    }

    public void setDashboard(FtcDashboard dashboard) {
        this.dashboard = dashboard;
        packet = new TelemetryPacket();
    }

    /**
     * Stops all the 4 drivetrain motors
     */
    public void idle() {
        frontLeft.set(0);
        frontRight.set(0);
        backLeft.set(0);
        backRight.set(0);
    }

    /**
     * Sets the gyroPIDF to some specific coeffs
     * @param gyroPID
     */
    public void setGyroPID(PID gyroPID) {
        this.gyroPID = gyroPID;
    }

    /**
     * Drives the robot following an specific angle
     * @param power motor's power
     * @param angle desired angle to follow
     * @param time drive time
     * @param pid pid controller
     */
    public void driveGyro(double[] power, double angle, double time, PID pid) {
        setGyroPID(pid);
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

            frontLeft.set(power[0] + correction);
            frontRight.set(power[1] - correction);
            backLeft.set(power[2] + correction);
            backRight.set(power[3] - correction);

            double error = Imu.getError(imu.getAngleNormalized(), angle);
            correction = gyroPID.calculate(error,0);
        }

        idle();
    }

    /**
     * Sets the encoderPIDF to some specific coeffs
     * @param encoderPID
     */
    public void setEncoderPID(PID encoderPID) {
        this.encoderPID = encoderPID;
    }

    /**
     * Drives the robot following an specific angle traveling an specified amount of centimeters
     * @param power motor's power
     * @param angle angle to follow
     * @param goal cm to move
     * @param gyroPID gyroPIDF controller
     * @param encoderPID encoderPIDF controller
     */
    public void driveEncoderGyro(double[] power, double angle, double goal, PID gyroPID, PID encoderPID) {
        setGyroPID(gyroPID);
        setEncoderPID(encoderPID);
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
        double relativePosition;
        double startingLeftPosition = -backLeft.getCurrentPosition();
        double startingRightPosition = backRight.getCurrentPosition();
        double ticksGoal = Drivetrain.cmToTicks(goal);

        // Relative Ticks
        while (!Thread.currentThread().isInterrupted()) {

            relativePosition = (Math.abs(-backLeft.getCurrentPosition() - startingLeftPosition) + Math.abs(backRight.getCurrentPosition() - startingRightPosition)) / 2.0;

            double encoderOutput = encoderPID.calculate(relativePosition, ticksGoal);
            double angleError = Imu.getError(imu.getAngleNormalized(), angle);
            double angleCorrection = gyroPID.calculate(angleError, 0);

            frontLeft.set(encoderOutput * power[0] - angleCorrection);
            frontRight.set(encoderOutput * power[1] + angleCorrection);
            backLeft.set(encoderOutput * power[2] - angleCorrection);
            backRight.set(encoderOutput * power[3] + angleCorrection);

            packet.put("Goal", ticksGoal);
            packet.put("Position", relativePosition);
            packet.put("Error", encoderPID.getError());
            packet.put("Output", encoderOutput);
            packet.put("pContrib", encoderPID.getPContrib());
            packet.put("iContrib", encoderPID.getIContrib());
            packet.put("dContrib", encoderPID.getDContrib());
            packet.put("setPoint", encoderPID.getSetPoint());

            dashboard.sendTelemetryPacket(packet);

            if (encoderPID.getError() < Drivetrain.cmToTicks(2))break;
        }
        idle();
    }

    /**
     * Sets the gyroPIDF to some specific coeffs
     * @param orientationPID pidf controller
     */
    public void setOrientationPIDF(PID orientationPID) {
        this.orientationPID = orientationPID;
    }

    public void setOrientationPID(PID orientationPID) {
        this.orientationPID = orientationPID;
    }

    /**
     * Turns the robot to an specific angle
     * @param power motor's power
     * @param angle angle to turn
     * @param pid pid controller
     */
    public void setOrientation(double power, double angle, PID pid) {
        setOrientationPID(pid);
        setOrientationC(power, angle);
    }


    /**
     * Turns the robot to an specific angle. It requires to have set a orientationPIDF controller.
     * @param power
     * @param angle
     */
    public void setOrientationC(double power, double angle) {

        ElapsedTime zeroErrorTime = new ElapsedTime();
        zeroErrorTime.reset();
        boolean firstZero = false;

        while (!Thread.currentThread().isInterrupted()) {

            double error = Imu.getError(imu.getAngleNormalized(), angle);
            double correction =  orientationPID.calculate(error,0);

            frontLeft.set(power * correction);
            frontRight.set(-power * correction);
            backLeft.set(power * correction);
            backRight.set(-power * correction);

            packet.put("Angle", imu.getAngleNormalized());
            packet.put("Error", error);
            packet.put("Output", correction);
            packet.put("pContrib", orientationPID.getPContrib());
            packet.put("iContrib", orientationPID.getIContrib());
            packet.put("dContrib", orientationPID.getDContrib());
            packet.put("setPoint", orientationPID.getSetPoint());

            dashboard.sendTelemetryPacket(packet);

            if (Math.abs(error) <= 0.6)
            {
                idle();
                break;
            }
            // Que se cumplan una serie de segundos y luego quebrar
        }
        idle();
    }
    /**
     * Turns the robot to an specific angle. It requires to have set a orientationPIDF controller.
     * @param power
     * @param angle
     */
    public void setOrientation(double power, double angle) {

        ElapsedTime zeroErrorTime = new ElapsedTime();
        zeroErrorTime.reset();
        boolean firstZero = false;

        while (!Thread.currentThread().isInterrupted()) {

            double error = Imu.getError(imu.getAngleNormalized(), angle);
            double correction =  orientationPID.calculate(error,0);

            frontLeft.set(power * correction);
            frontRight.set(-power * correction);
            backLeft.set(power * correction);
            backRight.set(-power * correction);

            packet.put("Angle Error", error);
            packet.put("Correction Output", correction);

            dashboard.sendTelemetryPacket(packet);

            if (Math.abs(error) <= 0.6)
            {
                if (!firstZero){
                    firstZero = true;
                    zeroErrorTime.reset();
                }
                if (zeroErrorTime.milliseconds() > 1000) {
                    break;
                }
            }
            else
            {
                firstZero = false;
            }
             // Que se cumplan una serie de segundos y luego quebrar
        }
        idle();
    }

    /**
     * follows the path of a predefined spline
     * @param spline spline
     * @param power motor's power
     */
    public void followPath(Spline spline, double power) {
        // Todo en centimetros
        double relativePosition;
        double startingPosition = Drivetrain.ticksToCm((backLeft.getCurrentPosition() + backRight.getCurrentPosition()) / 2.0);
        double startingHeading = spline.getHeading(0);

        setOrientation(power, startingHeading);

        // Should have really low power to avoid overshooting
        do {

            relativePosition = Drivetrain.ticksToCm((backLeft.getCurrentPosition() + backRight.getCurrentPosition()) / 2.0) - startingPosition;
            double newHeading = Imu.normalizeSplineAngle(spline.getHeading(relativePosition));
            double angleError = Imu.getError(imu.getAngleNormalized(), newHeading);
            double angleCorrection = orientationPID.calculate(angleError, 0);

            frontLeft.set(power + angleCorrection); // Maybe it should change depending of direction
            frontRight.set(power - angleCorrection);
            backLeft.set(power + angleCorrection);
            backRight.set(power - angleCorrection);


            packet.put("Position", relativePosition);
            packet.put("New Heading", newHeading);
            packet.put("Angle Error", angleError);
            packet.put("Correction Output", angleCorrection);

            dashboard.sendTelemetryPacket(packet);

        } while (!(relativePosition > spline.points[spline.points.length - 1].x) && !Thread.currentThread().isInterrupted());

        idle();

    }

    /**
     * Converts cm to motor ticks
     * @param cm cm
     * @return motor ticks
     */
    public static double cmToTicks(double cm) {
        return cm * Constants.TICKS_PER_REVOLUTION / Constants.WHEEL_CIRCUMFERENCE_CM;
    }

    /**
     * Converts motor ticks to cm
     * @param ticks ticks
     * @return motor ticks
     */
    public static double ticksToCm(double ticks) {
        return ticks * Constants.WHEEL_CIRCUMFERENCE_CM / Constants.TICKS_PER_REVOLUTION;
    }

}
