package org.firstinspires.ftc.teamcode.Voltrons.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Belt {

    CRServo beltDown;
    CRServo beltUp;

    double power;

    /**
     * Initialize Belt Class
     * @param beltDown CRServo representing the bottom part of the belt
     * @param beltUp CRServo representing the upper part of the belt
     */
    public Belt(CRServo beltDown, CRServo beltUp) {
        this.beltDown = beltDown;
        this.beltUp = beltUp;
    }

    /**
     * Makes the belt to go up
     */
    public void moveUp() {
        power = 1;
        beltDown.setPower(1);
        beltUp.setPower(1);
    }

    /**
     * Makes the belt to go down
     */
    public void moveDown() {
        power = -1;
        beltUp.setPower(-1);
        beltDown.setPower(-1);
    }

    /**
     * Stops the belt
     */
    public void stop() {
        power = 0;
        beltUp.setPower(0);
        beltDown.setPower(0);
    }

    /**
     * Set Belt's power to a desired one
     * @param power Belt's power
     */
    public void setPower(double power) {
        this.power = power;
        beltUp.setPower(power);
        beltDown.setPower(power);
    }

    /**
     * Inverts both CRServos to a desired direction
     * @param direction true -> inverted; false -> normal
     */
    public void invert(boolean direction) {
        if (!direction) {
            beltUp.setDirection(CRServo.Direction.FORWARD);
            beltDown.setDirection(CRServo.Direction.FORWARD);
        }
        else
        {
            beltUp.setDirection(CRServo.Direction.REVERSE);
            beltDown.setDirection(CRServo.Direction.REVERSE);
        }
    }

    /**
     * Returns the current belt power
     * @return power
     */
    public double getPower() {
        return power;
    }

}
