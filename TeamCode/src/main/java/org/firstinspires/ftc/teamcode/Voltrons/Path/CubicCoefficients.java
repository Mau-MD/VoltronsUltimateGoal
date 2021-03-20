package org.firstinspires.ftc.teamcode.Voltrons.Path;

import org.ejml.simple.SimpleMatrix;


import java.util.Arrays;

public class CubicCoefficients {

    static int matrixSize;

    /**
     * Object that requires the numer of functions to calculate the return matrix size
     * @param functions
     */
    public CubicCoefficients(int functions) {
        matrixSize = 4 * (functions);
    }

    /**
     * Returns the coefficients of a curve given a point
     * @param function number of functions
     * @param x x point value
     * @param y y point value
     * @return Two vectors, one that has the coefficients and other that stores y values
     */
    public VectorPair curve(int function, double x, double y) {
        // a + bx + cx^2 + dx^3
        double a, b, c, d;
        a = 1;
        b = x;
        c = x * x;
        d = x * x * x;

        double[] answer = new double[matrixSize];

        Arrays.fill(answer,0);
        answer[4 * function] = a;
        answer[4 * function + 1] = b;
        answer[4 * function + 2] = c;
        answer[4 * function + 3] = d;

        return new VectorPair(answer, y);
    }

    /**
     *  Returns the coefficients that satisfies start and final slope continuity.
     * @param function number of functions
     * @param x x point value
     * @param slope slope value
     * @return Two vectors, one that has the coefficients and other that stores y values
     */
    public VectorPair slope(int function, double x, double slope) {
        double b1, c1, d1;
        b1 = 1;
        c1 = x;
        d1 = 2 * x * x;

        double[] answer = new double[matrixSize];
        Arrays.fill(answer,0);

        answer[4 * function + 1] = b1;
        answer[4 * function + 2] = c1;
        answer[4 * function + 3] = d1;

        return new VectorPair(answer, slope);
    }

    /**
     * Returns the coefficients that satisfies C^1 continuity: Slope Match
     * @param function number of functions
     * @param x x point value
     * @return Two vectors, one that has the coefficients and other that stores y values
     */
    public VectorPair slopeMatch(int function, double x) {
        double b1, c1, d1, b2, c2, d2;
        b1 = 1;
        c1 = 2 * x;
        d1 = 3 * x * x;
        b2 = -1;
        c2 = -2 * x;
        d2 = -3 * x * x;

        double[] answer = new double[matrixSize];
        Arrays.fill(answer,0);

        answer[4 * function + 1] = b1;
        answer[4 * function + 2] = c1;
        answer[4 * function + 3] = d1;

        answer[4 * (function + 1) + 1] = b2;
        answer[4 * (function + 1) + 2] = c2;
        answer[4 * (function + 1) + 3] = d2;

        return new VectorPair(answer, 0);
    }

    /**
     *  Returns the coefficients that satisfies C^2 continuity: Curvature Match
     * @param function number of functions
     * @param x x point value
     * @return Two vectors, one that has the coefficients and other that stores y values
     */
    public VectorPair curvatureMatch(int function, double x) {
        double c1, d1, c2, d2;
        c1 = 2;
        d1 = 6 * x;
        c2 = -2;
        d2 = -6 * x;

        double[] answer = new double[matrixSize];
        Arrays.fill(answer,0);

        answer[4 * function + 2] = c1;
        answer[4 * function + 3] = d1;

        answer[4 * (function + 1) + 2] = c2;
        answer[4 * (function + 1) + 3] = d2;

        return new VectorPair(answer, 0);
    }
}
