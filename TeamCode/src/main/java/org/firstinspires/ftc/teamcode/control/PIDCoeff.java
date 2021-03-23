package org.firstinspires.ftc.teamcode.control;

public class PIDCoeff {

    public double kP;
    public double kD;
    public double kI;

    public PIDCoeff(double kP, double kD, double kI) {
        this.kP = kP;
        this.kD = kD;
        this.kI = kI;
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
}
