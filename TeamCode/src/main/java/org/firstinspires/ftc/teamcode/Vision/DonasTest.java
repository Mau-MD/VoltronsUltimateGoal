package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@TeleOp (name = "DonasTest", group = "Tests")
public class DonasTest extends LinearOpMode {

    OpenCvCamera phoneCam;
    RingPipeline visionPipeline;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.openCameraDevice();

        visionPipeline = new RingPipeline();
        phoneCam.setPipeline(visionPipeline);
        phoneCam.startStreaming(320,240,OpenCvCameraRotation.UPRIGHT);
        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Ring 1:",visionPipeline.ring1);
            telemetry.addData("Ring 4:",visionPipeline.ring4);
            telemetry.update();
        }
    }
}
