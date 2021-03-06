package org.firstinspires.ftc.teamcode.Voltrons.control;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {

    ElapsedTime elapsedTime = new ElapsedTime();

    public PIDICoeff coeff;

    double error;
    double lastTimeStamp;
    double errorSum;
    double lastError;
    double setPoint;

    double pContrib;
    double iContrib;
    double dContrib;

    public PID(double kP, double kI, double kD, double iLimit) {

        coeff = new PIDICoeff(kP, kI, kD, iLimit);

        elapsedTime.reset();

        lastTimeStamp = elapsedTime.seconds();
        error = 0;
        errorSum = 0;
        lastError = 0;
        setPoint = 0;

        pContrib = 0;
        iContrib = 0;
        dContrib = 0;
    }

    public PID(PIDICoeff coeff) {

        this.coeff = coeff;

        elapsedTime.reset();

        lastTimeStamp = elapsedTime.seconds();
        error = 0;
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
        error = 0;
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

    public void setCoeff(PIDICoeff coeff) {
        this.coeff = coeff;
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

    public double getError() { return error; }

    public PIDICoeff getCoeff() {
        return coeff;
    }

    public void resetErrorSum() {
        errorSum = 0;
    }

    public double getErrorSum() {
        return errorSum;
    }

    public double calculate(double cP, double sP) {
        error = sP - cP;
        double dt = elapsedTime.seconds() - lastTimeStamp;

        if (Math.abs(error) < coeff.iLimit) {
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
