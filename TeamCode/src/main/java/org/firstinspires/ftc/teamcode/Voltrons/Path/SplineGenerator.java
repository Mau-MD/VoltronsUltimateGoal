package org.firstinspires.ftc.teamcode.Voltrons.Path;


import org.ejml.simple.SimpleMatrix;
import java.util.Arrays;

public class SplineGenerator {

    // a + bx + cx^2 + dx^3 = y
    // Math normal x y format

    /**
     * Gets all the functions needed to create a spline based on n construct points. The matrix size will be
     * (number_of_functions - 1) * 4. Every index will have a function coefficient in the form
     * a1, b1, c1, d1, a2, b2, c2, d2, ... , an, bn, cn, dn. Every 4 coefficients representing a cubic function
     * in the form of a + bx + cx^2 + dx^3
     *
     * @param constructPoints An array of points where the spline will be generated
     * @param slope1 initial slope
     * @param slope2 final slope
     * @return SimpleMatrix that stores function coefficients
     */
    public static SimpleMatrix getSplineFunctions(Point[] constructPoints, double slope1, double slope2) {

        CubicCoefficients coeff = new CubicCoefficients( constructPoints.length - 1);
        double[][] tMatrix = new double[4 * (constructPoints.length - 1)][4 * (constructPoints.length - 1)];
        double[] yMatrix = new double[4 * (constructPoints.length - 1)];


        int currentRow = 0;

        for (int i = 0; i < constructPoints.length-1; i++) {
            if (i == 0) { // Start of the points
                VectorPair slope = coeff.slope(i, constructPoints[i].x, slope1);
                tMatrix[currentRow] = slope.vector;
                yMatrix[currentRow] = slope.result;
                currentRow++;
            }
            VectorPair curve1 = coeff.curve(i, constructPoints[i].x, constructPoints[i].y);
            tMatrix[currentRow] = curve1.vector;
            yMatrix[currentRow] = curve1.result;
            currentRow++;

            VectorPair curve2 = coeff.curve(i, constructPoints[i+1].x, constructPoints[i+1].y);
            tMatrix[currentRow] = curve2.vector;
            yMatrix[currentRow] = curve2.result;
            currentRow++;

            if (i == constructPoints.length-2) {
                VectorPair slope = coeff.slope(i,constructPoints[i+1].x, slope2);
                tMatrix[currentRow] = slope.vector;
                yMatrix[currentRow] = slope.result;
                currentRow++;
            }
            else {
                VectorPair slopeMatch = coeff.slopeMatch(i, constructPoints[i + 1].x);
                tMatrix[currentRow] = slopeMatch.vector;
                yMatrix[currentRow] = slopeMatch.result;
                currentRow++;

                VectorPair curveMatch = coeff.curvatureMatch(i, constructPoints[i + 1].x);
                tMatrix[currentRow] = curveMatch.vector;
                yMatrix[currentRow] = curveMatch.result;
                currentRow++;
            }
        }

        SimpleMatrix m = new SimpleMatrix(tMatrix);
        SimpleMatrix y = new SimpleMatrix(yMatrix.length,1, true, yMatrix);
        SimpleMatrix minv = m.invert();
        SimpleMatrix a = minv.mult(y);

        System.out.println(a);
        return a;
    }
}
