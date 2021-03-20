package org.firstinspires.ftc.teamcode.Voltrons.Path;


import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.integration.SimpsonIntegrator;
import org.apache.commons.math3.analysis.integration.TrapezoidIntegrator;
import org.apache.commons.math3.analysis.integration.UnivariateIntegrator;

public class CubicFunction {

    double a;
    double b;
    double c;
    double d;

    /**
     * Initializes a cubic polynomial in the form a + bx + cx^2 + dx^3
     * @param a a
     * @param b bx
     * @param c cx^2
     * @param d dx^3
     */
    public CubicFunction(double a, double b, double c, double d) {
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
    }

    /**
     * Evaluates the cubic function given a x value
     * @param x x point value
     * @return y result
     */
    public double evaluate(double x) {
        return (a + b * x + c * x * x + d * x * x * x);
    }

    /**
     * Gets the length of the cubic function given a range
     * @param low start of the range
     * @param high end of the range
     * @return length of the function
     */
    public double getLength(double low, double high) {
        UnivariateFunction function = x -> Math.sqrt(1 + (b + 2 * c * x + 3 * d * x * x) * (b + 2 * c * x + 3 * d * x * x));
        TrapezoidIntegrator integrator = new TrapezoidIntegrator();
        return integrator.integrate(10000,function,low,high);
    }

    /**
     * Returns the slope of the function given a x value
     * @param x x value
     * @return slope
     */
    public double getSlope(double x) {
        double c = this.c * 2, d = this.d * 3;
        return b + c * x + d * x * x;
    }

    /**
     * Returns the a coefficient of the cubic function
     * @return a
     */
    public double getA() {
        return a;
    }

    /**
     * Returns the a coefficient of the cubic function
     * @return b
     */
    public double getB() {
        return b;
    }

    /**
     * Returns the c coefficient of the cubic function
     * @return c
     */
    public double getC() {
        return c;
    }

    /**
     * Returns the d coefficient of the cubic function
     * @return d
     */
    public double getD() {
        return d;
    }

    /**
     * Returns the an array of 4 doubles that has all the four coefficients of the cubic function
     * @return coeffs array
     */
    public double[] getCoeff() {
        return new double[] {a,b,c,d};
    }

}
