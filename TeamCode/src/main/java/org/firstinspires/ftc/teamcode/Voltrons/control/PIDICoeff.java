package org.firstinspires.ftc.teamcode.Voltrons.control;

public class PIDICoeff {

    public double kP;
    public double kD;
    public double kI;
    public double iLimit;

    public PIDICoeff(double kP, double kI, double kD, double iLimit) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.iLimit = iLimit;
    }

    public double getP() {
        return kP;
    }

    public double getI() {
        return kI;
    }

    public double getD() {
        return kD;
    }

    public double getILimit() { return iLimit; }
}
