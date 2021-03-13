package org.firstinspires.ftc.teamcode.Voltrons.hardware;

import com.arcrobotics.ftclib.hardware.motors.Motor;

public class Launcher {

    Motor leftLauncher;
    Motor rightLauncher;

    double power;

    /**
     * Initializes the Launcher class
     * @param leftLauncher left motor
     * @param rightLauncher right motor
     */
    public Launcher(Motor leftLauncher, Motor rightLauncher) {
        this.leftLauncher = leftLauncher;
        this.rightLauncher = rightLauncher;
    }

    /**
     * Sets the power of both motors to a desired value
     * @param power power
     */
    public void setPower(double power) {
        this.power = power;
        leftLauncher.set(-power);
        rightLauncher.set(power);
    }

    /**
     * Inverts both motors to a desired direction
     * @param direction true -> inverted; false -> normal
     */
    public void invert(boolean direction) {
        leftLauncher.setInverted(direction);
        rightLauncher.setInverted(direction);
    }

    /**
     * Returns the current power of both motors
     * @return motor's power
     */
    public double getPower() {
        return power;
    }


}
