package frc.trigon.robot.utilities;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Maths {
    /**
     * Calculates the polynomial of the form ax^2 + bx + c.
     *
     * @param a the a in the polynomial
     * @param b the b in the polynomial
     * @param c the c in the polynomial
     * @param x the x in the polynomial
     * @return the result of the polynomial
     */
    public static double calculatePolynomial(double a, double b, double c, double x) {
        return (a * Math.pow(x, 2)) + (b * x) + c;
    }

    /**
     * Calculates the slope of a line between two points.
     *
     * @param firstPoint  the first point
     * @param secondPoint the second point
     * @return the slope
     */
    public static double calculateSlope(Translation2d firstPoint, Translation2d secondPoint) {
        return (firstPoint.getY() - secondPoint.getY()) / (firstPoint.getX() - secondPoint.getX());
    }

    /**
     * Calculates the angle between two points.
     *
     * @param firstPoint  the first point
     * @param secondPoint the second point
     * @return the angle
     */
    public static Rotation2d getAngleBetweenTranslations(Translation2d firstPoint, Translation2d secondPoint) {
        final double slope = calculateSlope(firstPoint, secondPoint);
        if (firstPoint.getX() > secondPoint.getX())
            return new Rotation2d(Math.atan(slope)).rotateBy(Rotation2d.fromRotations(0.5));
        if (!Double.isInfinite(slope) && !Double.isNaN(slope))
            return new Rotation2d(Math.atan(slope));

        if (firstPoint.getDistance(secondPoint) == 0)
            return Rotation2d.fromDegrees(0);
        if (firstPoint.getY() < secondPoint.getY())
            return Rotation2d.fromDegrees(90);

        return Rotation2d.fromDegrees(-90);
    }

    /**
     * Checks what's the closest number to the given number.
     *
     * @param toCheck the number to check which number is closer to it.
     * @param num1    the first number
     * @param num2    the second number
     * @return the closest number to the given number
     */
    public static double getClosestNumber(double toCheck, double num1, double num2) {
        final double num1Difference = Math.abs(toCheck - num1);
        final double num2Difference = Math.abs(toCheck - num2);
        if (num1Difference < num2Difference)
            return num1;
        return num2;
    }
}
