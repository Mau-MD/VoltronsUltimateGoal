package org.firstinspires.ftc.teamcode.Voltrons.Vision;

import com.acmerobotics.dashboard.config.Config;

import org.ejml.sparse.csc.mult.ImplMultiplicationWithSemiRing_DSCC;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

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

    public static Point startingPoint = new Point(118, 155);
    public static Point endPoint = new Point(175, 200);
    public static double ringYDelimiter = 190;

    public static double minArea = 1000;

    @Override
    public Mat processFrame(Mat input)
    {

        // Img processing
        Imgproc.cvtColor(input,YCrCb,Imgproc.COLOR_BGR2YCrCb);
        Core.extractChannel(YCrCb,Cb,2);
        Imgproc.threshold(Cb,tholdMat,150,255,Imgproc.THRESH_BINARY_INV);



        // Point BigSquarePoint = new Point((int)((BigSquare1.x + BigSqare2.x) / 2),(int)((BigSquare1.y + SmallSquare1.y) / 2));
        // Point SmallSquarePoint = new Point((int)((SmallSquare1.x + SmallSquare2.x) / 2),(int)((SmallSquare1.y + SmallSquare2.y) / 2));

        // Large Region
        double largeRegionBlackPixels = 0, largeRegionWhitePixels = 0;
        for (int y = (int) startingPoint.y; y < ringYDelimiter; y++) {
            for (int x = (int) startingPoint.x; x < endPoint.x; x++) {
                if (tholdMat.get(y, x)[0] == 0) { // Theres a black pixel == ring there
                    largeRegionBlackPixels++;
                }
                else {
                    largeRegionWhitePixels++;
                }
            }
        }

        double largeRegionTotal = largeRegionBlackPixels + largeRegionWhitePixels;
        double largeRegionBlackAverage = largeRegionBlackPixels / largeRegionTotal;
        double largeRegionWhiteAverage = largeRegionWhitePixels / largeRegionTotal;

        // Small Region
        double smallRegionBlackPixels = 0, smallRegionWhitePixels = 0;
        for (int y = (int) ringYDelimiter; y < endPoint.y; y++) {
            for (int x = (int) startingPoint.x; x < endPoint.x; x++) {
                if (tholdMat.get(y, x)[0] == 0) { // There's a black pixel == ring there
                    smallRegionBlackPixels++;
                }
                else {
                    smallRegionWhitePixels++;
                }
            }
        }

        double smallRegionTotal = smallRegionBlackPixels + smallRegionWhitePixels;
        double smallRegionBlackAverage = smallRegionBlackPixels / smallRegionTotal;
        double smallRegionWhiteAverage = smallRegionWhitePixels / smallRegionTotal;


        ring4 = 255;
        ring1 = 255;

        if (largeRegionBlackAverage > largeRegionWhiteAverage) {
            // Four Ring On
            ring4 = 0;
        }

        if (smallRegionBlackPixels > smallRegionWhiteAverage) {
            // First Rin On
            ring1 = 0;
        }


        Imgproc.rectangle(
                input,
                startingPoint,
                endPoint,
                GRAY,
                1
        );

        Imgproc.rectangle(
                input,
                new Point(startingPoint.x, ringYDelimiter),
                endPoint,
                GRAY,
                1
        );

        if (ring1 == 0 && ring4 == 0) {
            Imgproc.rectangle(
                    input,
                    startingPoint,
                    endPoint,
                    GREEN,
                    1
            );

            Imgproc.rectangle(
                    input,
                    new Point(startingPoint.x, ringYDelimiter),
                    endPoint,
                    GREEN,
                    1
            );
        }
        else if (ring1 == 0) {
            Imgproc.rectangle(
                    input,
                    new Point(startingPoint.x, ringYDelimiter),
                    endPoint,
                    GREEN,
                    1
            );
        }


        return input;
    }


}
