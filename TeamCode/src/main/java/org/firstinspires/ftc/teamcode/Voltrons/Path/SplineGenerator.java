package org.firstinspires.ftc.teamcode.Voltrons.Path;

import org.ejml.simple.SimpleMatrix;

public class SplineGenerator {

    // a + bx + cx^2 + dx^3 = y
    // Math normal x y format
    public static SimpleMatrix getFunction(Point[] constructPoints, double slope1, double slope2) {

        CubicCoefficients coeff = new CubicCoefficients(constructPoints.length - 1);
        double[][] tMatrix = new double[4 * (constructPoints.length - 1)][4 * (constructPoints.length - 1)];
        double[] yMatrix = new double[4 * (constructPoints.length - 1)];


        int currentRow = 0;

        for (int i = 0; i < constructPoints.length-1; i++) {
            if (i == 0) { // Start of the points
                VectorPair slope = coeff.slope(0, constructPoints[i].x, slope1);
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
                VectorPair slope = coeff.slope(i,constructPoints[i].x, slope2);
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
    }
}
