package org.firstinspires.ftc.teamcode.Voltrons.control;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {

    ElapsedTime elapsedTime = new ElapsedTime();

    PIDCoeff coeff;
    double iLimit;

    double lastTimeStamp;
    double errorSum;
    double lastError;
    double setPoint;

    double pContrib;
    double iContrib;
    double dContrib;

    public PID(double kP, double kI, double kD, double iLimit) {

        coeff = new PIDCoeff(kP, kI, kD);
        this.iLimit = iLimit;

        elapsedTime.reset();

        lastTimeStamp = elapsedTime.seconds();
        errorSum = 0;
        lastError = 0;
        setPoint = 0;

        pContrib = 0;
        iContrib = 0;
        dContrib = 0;
    }

    public PID(PIDCoeff coeff, double iLimit) {

        this.coeff = coeff;
        this.iLimit = iLimit;

        elapsedTime.reset();

        lastTimeStamp = elapsedTime.seconds();
        errorSum = 0;
        lastError = 0;
        setPoint = 0;

        pContrib = 0;
        iContrib = 0;
        dContrib = 0;
    }

    public void reset() {
        elapsedTime.reset();

        lastTimeStamp = elapsedTime.seconds();
        errorSum = 0;
        lastError = 0;
        setPoint = 0;

        pContrib = 0;
        iContrib = 0;
        dContrib = 0;
    }

    public void setP(double kP) {
        coeff.kP = kP;
    }

    public void setD(double kD) {
        coeff.kD = kD;
    }

    public void setI(double kI) {
        coeff.kI = kI;
    }

    public void setCoeff(PIDCoeff coeff) {
        this.coeff = coeff;
    }

    public void setILimit(double iLimit) {
        this.iLimit = iLimit;
    }

    public void setSetPoint(double sP) {
        setPoint = sP;
    }

    public double getSetPoint() {
        return setPoint;
    }

    public double getPContrib() {
        return pContrib;
    }

    public double getIContrib() {
        return iContrib;
    }

    public double getDContrib() {
        return dContrib;
    }

    public double calculate(double cP, double sP) {
        double error = sP - cP;
        double dt = elapsedTime.seconds() - lastTimeStamp;

        if (Math.abs(error) < iLimit) {
            errorSum += error * dt;
        }

        double errorRate = (error - lastError) / dt;

        pContrib = coeff.kP * error;
        iContrib = coeff.kI * errorSum;
        dContrib = coeff.kD * errorRate;

        lastTimeStamp = elapsedTime.seconds();
        lastError = error;

        return pContrib + iContrib + dContrib;
    }

}
