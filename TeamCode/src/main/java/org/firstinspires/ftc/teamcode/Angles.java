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

    /**
     * NoSpinAngle adds a limited angle (from IMU) to a continuous angle to avoid the spin bug.
     * @param continuousAngle the continuous angle
     * @param newAngle the limited angle from -2*PI to 2*PI
     * @return the updates continuous angle
     */
    public double NoSpinAngle(double continuousAngle, double newAngle) {
        double deltaAngle = newAngle - continuousAngle;

        if (deltaAngle > Math.PI) deltaAngle -= 2 * Math.PI;
        else if (deltaAngle < -Math.PI) deltaAngle += 2 * Math.PI;
        return deltaAngle;
    }
}
