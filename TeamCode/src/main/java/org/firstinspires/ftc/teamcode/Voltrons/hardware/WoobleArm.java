package org.firstinspires.ftc.teamcode.Voltrons.hardware;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.util.Range;

public class WoobleArm {

    Motor arm;
    SimpleServo hand;
    boolean open;
    double min, max;

    PIDFController pidf;
    double[] pidfCoeff;
    double goal;
    double armMin, armMax;
    boolean active = true;
    double cpr;
    /**
     * Initializes the WobbleArm Class
     * @param arm Motor that controls the arm
     * @param hand Servo that controls the hand
     * @param pidf PIDF Class that ensures a precise movement of the arm. It should have the Coeffs already set.
     * @param armMin Minimum degree of the arm
     * @param armMax Maximum degree of the arm
     * @param cpr Counts per Revolution of the motor
     */
    public WoobleArm(Motor arm, SimpleServo hand, PIDFController pidf, double armMin, double armMax, double cpr) {
        this.arm = arm;
        this.hand = hand;
        this.pidf = pidf;
        this.armMin = armMin;
        this.armMax = armMax;
        this.cpr = cpr;
    }

    /**
     * Opens the wobble hand
     */
    public void openHand() {
        open = true;
        hand.setPosition(1);
    }

    /**
     * Closes the wobble hand
     */
    public void closeHand() {
        open = false;
        hand.setPosition(0);
    }

    /**
     * Sets the hand to a desired position
     * @param position position between 0 and 1
     */
    public void setPosition(double position) {
        hand.setPosition(position);
    }

    /**
     * Set the range of movement of the servo
     * @param min minimum value (in degrees)
     * @param max maximum value (in degrees)
     */
    public void setRange(double min, double max) {
        hand.setRange(min, max);
    }

    /**
     * Returns the current position of the hand
     * @return current position
     */
    public double getPosition() {
        return hand.getPosition();
    }

    /**
     * Gets the range of motion of the servos
     * @return range
     */
    public double getRange() {
        return max - min;
    }

    /**
     * Returns if the hand is currently open
     * @return true or false
     */
    public boolean isOpen() {
        return open;
    }
    // PID

    /**
     * Set's the PIDF Coeff
     * @param kp proportional constant
     * @param kd derivative constant
     * @param ki integral constant
     * @param kf friction constant
     */
    public void setPIDF(double kp, double kd, double ki, double kf) {
        pidf.setPIDF(kp,kd,ki,kf);
        pidfCoeff = new double[]{kp, kd, ki, kf};
    }

    /**
     * Set the new goal to follow
     * @param goal new objective goal
     */
    public void setGoal(double goal) {
        goal = Range.clip(goal, armMin, armMax);
        this.goal = goal;
        pidf.setSetPoint(goal); // Objetivo
    }

    /**
     * Set the arm range to a fixed degree
     * @param min Minimum degree of the arm
     * @param max Maximum degree of the arm
     */
    public void setArmRange(double min, double max) {
        armMin = min;
        armMax = max;
    }

    /**
     * Makes the wobble arm to go to the desired position
     */
    public void goToGoal() {
        if (!active)return;
        double output = pidf.calculate(arm.getCurrentPosition() * 360.0 / cpr); // Conversion to degrees
        arm.set(output);
    }

    /**
     * Deactivates PIDF Calculations
     */
    public void deactive() {
        active = false;
    }

    /**
     * Activates PIDF Calculations
     */
    public void activate() {
        active = true;
    }

    /**
     * Get the current goal position
     * @return current goal
     */
    public double getGoal() {
        return goal;
    }

    /**
     * Get the current PIDF Coeffs
     * @return an array of four elements with the Coeffs
     */
    public double[] getPIDF() {
        return pidfCoeff;
    }








}
