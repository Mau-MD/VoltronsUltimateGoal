package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class RingPipeline extends OpenCvPipeline {



    // Working Varibales
    Mat Cb = new Mat();
    Mat YCrCb = new Mat();
    Mat tholdMat = new Mat();

    Scalar GRAY = new Scalar(220,220,220);
    Scalar GREEN = new Scalar(0,255,0);

    public int ring1;
    public int ring4;

    public static Point BigSquare1 = new Point(120,150);
    public static Point BigSquare2 = new Point(150,172);

    public static Point SmallSquare1 = new Point(120,165);
    public static Point SmallSquare2 = new Point(150,172);

    @Override
    public Mat processFrame(Mat input)
    {

        // Img processing
        Imgproc.cvtColor(input,YCrCb,Imgproc.COLOR_BGR2YCrCb);
        Core.extractChannel(YCrCb,Cb,2);
        Imgproc.threshold(Cb,tholdMat,150,255,Imgproc.THRESH_BINARY_INV);

        // Drawing Points
        int BigSquarePointX = (int)((BigSquare1.x + BigSquare2.x) / 2);
        int BigSquarePointY = (int)((BigSquare1.y + SmallSquare1.y) / 2);

        int SmallSquarePointX = (int)((SmallSquare1.x + SmallSquare2.x) / 2);
        int SmallSquarePointY = (int)((SmallSquare1.y + SmallSquare2.y) / 2);

        // Point BigSquarePoint = new Point((int)((BigSquare1.x + BigSqare2.x) / 2),(int)((BigSquare1.y + SmallSquare1.y) / 2));
        // Point SmallSquarePoint = new Point((int)((SmallSquare1.x + SmallSquare2.x) / 2),(int)((SmallSquare1.y + SmallSquare2.y) / 2));

        double bigSquarePointValues[] = tholdMat.get(BigSquarePointY,BigSquarePointX);
        double smallSquarePointValues[] = tholdMat.get(SmallSquarePointY,SmallSquarePointX);

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
                new Point(BigSquarePointX,BigSquarePointY),
                2,
                GRAY,
                1
        );

        // Small Square Point
        Imgproc.circle(
                input,
                new Point(SmallSquarePointX,SmallSquarePointY),
                2,
                GRAY,
                1
        );

        // Change colors if the pipeline detected something

        if (ring1 == 0 && ring4 == 0)
        {
            Imgproc.rectangle(
                    input,
                    BigSquare1,
                    BigSquare2,
                    GREEN,
                    1
            );
            Imgproc.circle(
                    input,
                    new Point(BigSquarePointX,BigSquarePointY),
                    2,
                    GREEN,
                    1
            );
        }
        if (ring1 == 0)
        {
            Imgproc.rectangle(
                    input,
                    SmallSquare1,
                    SmallSquare2,
                    GREEN,
                    1
            );
            Imgproc.circle(
                    input,
                    new Point(SmallSquarePointX,SmallSquarePointY),
                    2,
                    GREEN,
                    1
            );
        }

        return input;
    }


}
