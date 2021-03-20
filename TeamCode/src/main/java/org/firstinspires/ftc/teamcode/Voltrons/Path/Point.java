package org.firstinspires.ftc.teamcode.Voltrons.Path;

public class Point {

    public double x;
    public double y;

    /**
     * Point class that stores x and y values
     * @param x
     * @param y
     */
    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Returns the distance between two points
     * @param a First point
     * @param b Second Point
     * @return distance between two points
     */
    public static double getDistance(Point a, Point b) {
        return Math.sqrt((b.y - a.y) * (b.y - a.y) + (b.x - a.x) * (b.x - a.x));
    }

}