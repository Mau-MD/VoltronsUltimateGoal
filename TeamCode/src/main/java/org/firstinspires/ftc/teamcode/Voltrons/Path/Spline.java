package org.firstinspires.ftc.teamcode.Voltrons.Path;


import android.util.Pair;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Voltrons.Constants;

import java.util.ArrayList;
import java.util.List;

public class Spline {

    List<CubicFunction> functions;
    public Point[] points;
    double[] prefixSum;
    double totalLength;

    /**
     * Initializes a spline objects given an array of points, the beginning slope and the end slope
     * @param points array of points
     * @param slope1 beginning slope
     * @param slope2 end slope
     */
    public Spline(Point[] points, double slope1, double slope2) {
        SimpleMatrix coeff = SplineGenerator.getSplineFunctions(points, slope1, slope2);
        List<CubicFunction> functions = new ArrayList<>();

        for (int i = 0; i < points.length - 1; i ++) {
            functions.add(new CubicFunction(coeff.get(i * 4), coeff.get(i * 4 + 1), coeff.get(i * 4 + 2), coeff.get(i * 4 + 3)));
        }

        this.points = points;
        this.functions = functions;
        prefixSum = new double[functions.size() + 1];
        totalLength = getTotalLength();
    }

    /**
     * Returns the total length of the spline
     * @return total length
     */
    private double getTotalLength() {
        double totalLength = 0;
        prefixSum[0] = 0;
        System.out.print(prefixSum[0] + " ");
        for (int i = 0;i < functions.size(); i++)
        {
            totalLength += functions.get(i).getLength(points[i].x, points[i+1].x);
            prefixSum[i+1] = totalLength;
            System.out.print(prefixSum[i+1] + " ");
        }
        return totalLength;
    }

    /**
     * Gets the function number based on a distance
     * @param x distance
     * @return number of function
     */
    private Pair<CubicFunction, Integer> getFunction(double x) {
        for (int i = 0; i < functions.size(); i ++) {
            /*
            if (x >= points[i].x && x <= points[i+1].x) {
                return new Pair<>(functions[i], i);
            }
             */
            if (x < prefixSum[i+1]) {
                return new Pair<>(functions.get(i), i);
            }
        }
        return new Pair<>(functions.get(functions.size()-1), functions.size()-1);
    }

    /**
     * Returns the coordinates of a function given a distance. Binary search oh yeah!
     * @param function function
     * @param distance current distance
     * @param _low lower bound of search aka. starting point
     * @param _high upper bound of search aka. final point
     * @return a pair of x and y coordinates of the function
     */
    private Point getCoordinates(CubicFunction function, double distance, double _low, double _high) {
        double low = _low, high = _high, mid;
        while (high - low >= 1e-9) {
            mid = (high + low) / 2;
            if (distance > function.getLength(_low, mid)) {
                low = mid;
            }
            else {
                high = mid;
            }
        }
        return new Point(low, function.evaluate(low));
    }

    /**
     * Gets the current angle of the function given a distance
     * @param x current distance
     * @return current angle
     */
    public double getHeading(double x) {

        Pair<CubicFunction, Integer> currentFunction = getFunction(x);
        System.out.println("Current Function " + currentFunction.second);
        double relativeLength = x - prefixSum[currentFunction.second]; // Real Length
        System.out.println("Relative Length: " + relativeLength);
        Point currentCoordinate = getCoordinates(
                currentFunction.first,
                relativeLength,
                points[currentFunction.second].x,
                points[currentFunction.second+1].x
        );
        //double absoluteXCoordinate = currentCoordinate.x + points[currentFunction.getValue()].x;
        System.out.println("Relative X: " + currentCoordinate.x);
        //System.out.println("Absolute X: " + absoluteXCoordinate);
        double slope = currentFunction.first.getSlope(currentCoordinate.x);
        return Math.toDegrees(Math.atan(slope));
    }


    /**
     * Converts computer-based coordinates to math-based coordinates
     * @param coordinate computer-based coordinate (x right, y down)
     * @return math-based coordinates (x right, y up)
     */
    public static Point toMathCoordinates(Point coordinate) {
        return new Point(coordinate.x, Constants.FIELD_HEIGHT_CM - coordinate.y);
    }

}
