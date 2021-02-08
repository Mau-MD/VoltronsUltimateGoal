package org.firstinspires.ftc.teamcode.tutorial;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp (name = "ringDetector", group = "Tests")
public class ringDetector extends LinearOpMode {

    OpenCvCamera phoneCam;

    // CONSTANTS

    final int X_LEFT = 1;
    final int X_RIGHT = 2;
    final int Y_UP = 1;
    final int Y_MIDDLE = 2;
    final int Y_DOWN = 3;



    @Override
    public void runOpMode()
    {
        // Camera Init
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();

        // Loading pipeline
        RingPipeline visionPipeline = new RingPipeline();
        phoneCam.setPipeline(visionPipeline);

        // Start streaming the pipeline
        phoneCam.startStreaming(320,240,OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        while (opModeIsActive())
        {
            // Get data from the pipeline and output it to the telemetry. This are the variables you are going to work with.
            telemetry.addData("Ring 1:",visionPipeline.ring1); // Will return 0 if there is 1 ring, otherwise 1
            telemetry.addData("Ring 4:",visionPipeline.ring4); // Will return 0 if there is 4 rings, otherwise 1
            telemetry.update();
        }
    }

    // Pipeline class
    class RingPipeline extends OpenCvPipeline {


        // Working Mat variables
        Mat YCrCb = new Mat(); // This will store the whole YCrCb channel
        Mat Cb = new Mat(); // This will store the Cb Channel (part from YCrCb)
        Mat tholdMat = new Mat(); // This will store the threshold

        // Drawing variables
        Scalar GRAY = new Scalar(220, 220, 220); // RGB values for gray.
        Scalar GREEN = new Scalar(0, 255, 0); // RGB values for green.

        // Variables that will store the results of our pipeline
        public int ring1;
        public int ring4;

        // Space which we will annalise data
        public Point BigSquare1 = new Point(X_LEFT, Y_UP);
        public Point BigSquare2 = new Point(X_RIGHT, Y_DOWN);

        public Point SmallSquare1 = new Point(X_LEFT, Y_MIDDLE);
        public Point SmallSquare2 = new Point(X_RIGHT, Y_DOWN);

        @Override
        public Mat processFrame(Mat input) {

            // Img processing
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_BGR2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
            Imgproc.threshold(Cb, tholdMat, 150, 255, Imgproc.THRESH_BINARY_INV);

            // Drawing Points
            int BigSquarePointX = (int) ((BigSquare1.x + BigSquare2.x) / 2);
            int BigSquarePointY = (int) ((BigSquare1.y + SmallSquare1.y) / 2);

            int SmallSquarePointX = (int) ((SmallSquare1.x + SmallSquare2.x) / 2);
            int SmallSquarePointY = (int) ((SmallSquare1.y + SmallSquare2.y) / 2);

            // Point BigSquarePoint = new Point((int)((BigSquare1.x + BigSqare2.x) / 2),(int)((BigSquare1.y + SmallSquare1.y) / 2));
            // Point SmallSquarePoint = new Point((int)((SmallSquare1.x + SmallSquare2.x) / 2),(int)((SmallSquare1.y + SmallSquare2.y) / 2));

            double[] bigSquarePointValues = tholdMat.get(BigSquarePointY, BigSquarePointX);
            double[] smallSquarePointValues = tholdMat.get(SmallSquarePointY, SmallSquarePointX);

            ring4 = (int) bigSquarePointValues[0];
            ring1 = (int) smallSquarePointValues[0];

            // Big Square
            Imgproc.rectangle(
                    input,
                    BigSquare1,
                    BigSquare2,
                    GRAY,
                    1
            );

            // Small Square
            Imgproc.rectangle(
                    input,
                    SmallSquare1,
                    SmallSquare2,
                    GRAY,
                    1
            );

            // Big Square Point
            Imgproc.circle(
                    input,
                    new Point(BigSquarePointX, BigSquarePointY),
                    2,
                    GRAY,
                    1
            );

            // Small Square Point
            Imgproc.circle(
                    input,
                    new Point(SmallSquarePointX, SmallSquarePointY),
                    2,
                    GRAY,
                    1
            );

            // Change colors if the pipeline detected something

            if (ring1 == 0 && ring4 == 0) {
                Imgproc.rectangle(
                        input,
                        BigSquare1,
                        BigSquare2,
                        GREEN,
                        1
                );
                Imgproc.circle(
                        input,
                        new Point(BigSquarePointX, BigSquarePointY),
                        2,
                        GREEN,
                        1
                );
            }
            if (ring1 == 0) {
                Imgproc.rectangle(
                        input,
                        SmallSquare1,
                        SmallSquare2,
                        GREEN,
                        1
                );
                Imgproc.circle(
                        input,
                        new Point(SmallSquarePointX, SmallSquarePointY),
                        2,
                        GREEN,
                        1
                );
            }

            return input;
        }
    }


}

