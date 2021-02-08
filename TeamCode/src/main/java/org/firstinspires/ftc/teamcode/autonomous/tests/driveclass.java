package org.firstinspires.ftc.teamcode.autonomous.tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Drive;

@Autonomous(name="DriveClassTest", group = "Tests")
public class driveclass extends LinearOpMode
{
    BNO055IMU imu;
    DcMotor lf,rf,lb,rb;

    @Override
    public void runOpMode()
    {


        Drive drive = new Drive(lf,rf,lb,rb,imu);

        waitForStart();
        if (opModeIsActive())
        {
            drive.driveTimedGyro(Drive.direction.FORWARD, 1, 1500, 90);
        }
    }
}
