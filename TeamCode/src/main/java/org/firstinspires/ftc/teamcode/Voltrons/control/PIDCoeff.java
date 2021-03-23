package org.firstinspires.ftc.teamcode.Voltrons.control;

public class PIDCoeff {

    public double kP;
    public double kD;
    public double kI;

    public PIDCoeff(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
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
