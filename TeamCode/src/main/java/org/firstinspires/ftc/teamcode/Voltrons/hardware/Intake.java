package org.firstinspires.ftc.teamcode.Voltrons.hardware;

import com.arcrobotics.ftclib.hardware.motors.Motor;

public class Intake {

    public Motor intake;
    double power;

    /**
     * Makes an intake class
     * @param intake intake motor
     */
    public Intake (Motor intake) {
        this.intake = intake;
    }

    /**
     * Sets the power to a desired value
     * @param power power
     */
    public void setPower(double power) {
        this.power = power;
        intake.set(power);
    }

    /**
     * Inverts the motor to a desired value
     * @param direction true -> inverted ; false -> normal
     */
    public void invert(boolean direction){
        intake.setInverted(direction);
    }

    /**
     * Returns the current power of the intake
     * @return intake's power
     */
    public double getPower() {
        return power;
    }
}
