package org.firstinspires.ftc.teamcode;

public class Angles {
    /**
     * Calculates the shortest distance to turn from angle A to angle B.
     * @param a Start angle in radians
     * @param b Target angle in radians
     * @return Shortest angle difference (-PI to PI)
     */
    public static double shortestAngleDifference(double a, double b) {
        double difference = a - b;
        // Normalize to (-PI, PI]
        while (difference <= -Math.PI) difference += 2.0*Math.PI;
        while (difference > Math.PI) difference -= 2.0*Math.PI;
        return difference;
    }
}
