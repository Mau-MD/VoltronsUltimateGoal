package org.firstinspires.ftc.teamcode.Voltrons.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.drive.Drive;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
    private double gyroKp;
    private PID orientationPID;

    private FtcDashboard dashboard;
    private TelemetryPacket packet;

    private double slopePercentage;
    private double minVel = 0.2;

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
     * @param gyroKp gyroKp
     */
    public void setGyroKp(double gyroKp) {
        this.gyroKp = gyroKp;
    }

    /**
     * Drives the robot following an specific angle
     * @param power motor's power
     * @param angle desired angle to follow
     * @param time drive time
     * @param gyroKp gyroKp
     */
    public void driveGyro(double[] power, double angle, double time, double gyroKp) {
        setGyroKp(gyroKp);
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
            correction = gyroKp * error;
        }

        idle();
    }


    public void setSlopeLimit(double percentage) {
        slopePercentage = percentage;
    }

    public double getSlopeMultiplier(double distance, double totalDistance) {
        double maxDistance = totalDistance * slopePercentage;
        if (distance > maxDistance)return 1;
        double output = distance / maxDistance;
        output = Math.max(output, minVel);
        // 1 -> max
        // x -> distance
        return output;
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
     * @param gyroKp gyroKp
     * @param encoderPID encoderPIDF controller
     */
    public void driveEncoderGyro(double[] power, double angle, double goal, double gyroKp, PID encoderPID) {
        setGyroKp(gyroKp);
        setEncoderPID(encoderPID);
        driveEncoderGyro(power, angle, goal);
    }

    public void driveEncoderGyro(double[] power, double angle, double goal, double slopePercentage)
    {
        setSlopeLimit(slopePercentage);
        driveEncoderGyro(power, angle, goal);
    }


    public void driveEncoderSimple(double[] power, double goal) {
        double startingLeftPosition = -backLeft.getCurrentPosition();
        double startingRightPosition = backRight.getCurrentPosition();
        double ticksGoal = Drivetrain.cmToTicks(goal);

        while (!Thread.currentThread().isInterrupted()) {
            double relativePosition = (Math.abs(-backLeft.getCurrentPosition() - startingLeftPosition) + Math.abs(backRight.getCurrentPosition() - startingRightPosition)) / 2.0;

            frontLeft.set(power[0]);
            frontRight.set(power[1]);
            backLeft.set(power[2]);
            backRight.set(power[3]);

            if (relativePosition > ticksGoal) {
                break;
            }
        }

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

        // setOrientation(Math.abs(power[0]), angle);

        // Relative Ticks
        while (!Thread.currentThread().isInterrupted()) {

            relativePosition = (Math.abs(-backLeft.getCurrentPosition() - startingLeftPosition) + Math.abs(backRight.getCurrentPosition() - startingRightPosition)) / 2.0;

            double encoderOutput = encoderPID.calculate(relativePosition, ticksGoal);
            double angleError = Imu.getError(imu.getAngleNormalized(), angle);
            double angleCorrection = gyroKp * angleError;

            double slopeOutput = getSlopeMultiplier(relativePosition, ticksGoal);

            encoderOutput = Range.clip(encoderOutput,-1,1);

            frontLeft.set(slopeOutput * encoderOutput * power[0] + angleCorrection);
            frontRight.set(slopeOutput * encoderOutput * power[1] - angleCorrection);
            backLeft.set(slopeOutput * encoderOutput * power[2] + angleCorrection);
            backRight.set(slopeOutput * encoderOutput * power[3] -   angleCorrection);

            packet.put("Angle Error", angleError);
            packet.put("Angle Correction", angleCorrection);

            packet.put("Left Front", slopeOutput * encoderOutput * power[0] - angleCorrection);
            packet.put("Right Front", slopeOutput * encoderOutput * power[1] + angleCorrection);
            packet.put("Left Back", slopeOutput * encoderOutput * power[2] - angleCorrection);
            packet.put("Right Back", slopeOutput * encoderOutput * power[3] + angleCorrection);

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

            if (Math.abs(error) <= 2)
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

            if (Math.abs(error) <= 2)
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
        double startingLeftPosition = Drivetrain.ticksToCm(-backLeft.getCurrentPosition());
        double startingRightPosition = Drivetrain.ticksToCm(backRight.getCurrentPosition());
        double startingHeading = Imu.normalizeSplineAngle(spline.getHeading(1));

        packet.put("Starting Heading", startingHeading);
        dashboard.sendTelemetryPacket(packet);

        setOrientation(power, startingHeading);

        // Should have really low power to avoid overshooting. 0.5 seems right
        do {

            relativePosition = (Math.abs(Drivetrain.ticksToCm(-backLeft.getCurrentPosition()) - startingLeftPosition) + Math.abs(Drivetrain.ticksToCm(backRight.getCurrentPosition()) - startingRightPosition)) / 2.0;
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

        } while (relativePosition < spline.totalLength && !Thread.currentThread().isInterrupted());

        idle();

    }

    public void followPathReverse(Spline spline, double power) {
        // Everything should be reversed
        double relativePosition;
        double startingLeftPosition = Drivetrain.ticksToCm(-backLeft.getCurrentPosition());
        double startingRightPosition = Drivetrain.ticksToCm(backRight.getCurrentPosition());
        double startingHeading = Imu.normalizeSplineAngle(spline.getHeading(spline.totalLength - 1));

        packet.put("Starting Heading", startingHeading);
        dashboard.sendTelemetryPacket(packet);

        setOrientation(power, startingHeading);

        // Should have really low power to avoid overshooting. 0.5 seems right
        do {

            relativePosition = (Math.abs(Drivetrain.ticksToCm(-backLeft.getCurrentPosition()) - startingLeftPosition) + Math.abs(Drivetrain.ticksToCm(backRight.getCurrentPosition()) - startingRightPosition)) / 2.0;
            double newHeading = Imu.normalizeSplineAngle(spline.getHeading(spline.totalLength - relativePosition));
            double angleError = Imu.getError(imu.getAngleNormalized(), newHeading);
            double angleCorrection = orientationPID.calculate(angleError, 0);

            frontLeft.set(-power + angleCorrection); // Maybe it should change depending of direction
            frontRight.set(-power - angleCorrection);
            backLeft.set(-power + angleCorrection);
            backRight.set(-power - angleCorrection);


            packet.put("Position", relativePosition);
            packet.put("New Heading", newHeading);
            packet.put("Angle Error", angleError);
            packet.put("Correction Output", angleCorrection);

            dashboard.sendTelemetryPacket(packet);

        } while (relativePosition < spline.totalLength && !Thread.currentThread().isInterrupted());

        idle();

    }

    public void drive(double[] power, double time) {
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        while (elapsedTime.milliseconds() < time) {
            frontLeft.set(power[0]); // Maybe it should change depending of direction
            frontRight.set(power[1]);
            backLeft.set(power[2]);
            backRight.set(power[3]);
        }
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
