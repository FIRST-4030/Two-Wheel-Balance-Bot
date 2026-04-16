package org.firstinspires.ftc.teamcode;

/**
 * Two Wheel Robot Odometry class.
 * Keeps track of a two-wheeled balancing robot's position using wheel encoders.
 * Running Average is used to smooth position and velocity data.
 * Robot pitch may move the encoders but not the change robot position, and
 * pitch may move the robot without changing the encoders.
 */
public class TWBOdometry {
    // Robot parameters
    private final double wheelBase; // distance between the wheels
    private final double wheelCircumference;  // wheel circumference
    
    private double lastPitch;

    // Position and orientation
    private double newLeftDistance, newRightDistance;

    private double x = 0;
    private double y = 0;
    private double s = 0;  // total distance regardless of direction
    private double theta = 0; // orientation angle in radians (-pi to pi)
    private double noNormalTheta = 0; // continuous orientation angle

    private double linearVelocity = 0.0;
    final private RunningAverageArray veloAvg;   // Running average of linear velocity
    
    final private RunningAverageArray leftDistAvg; // Running average of left encoder
    final private RunningAverageArray rightDistAvg; // Running average of left encoder

    /**
     * Constructor,  provide wheel base and wheel dia in mm, initialPitch in degrees
     * @param Nvelo The size of the running average array for linear velocity of bot
     * @param Ndist The size of the running average array for left and right distance
      */
    public TWBOdometry(double wheelBase, double wheelDia, double initialPitch,
                       int Nvelo, int Ndist) {
        this.wheelBase = wheelBase;
        this.wheelCircumference = wheelDia*Math.PI; // convert diameter to circumference
        this.lastPitch = initialPitch; // robots initial pitch, probably not zero

        // initialize the running averages with zeros to smooth out the startup
        this.veloAvg = new RunningAverageArray(Nvelo,true);
        this.leftDistAvg = new RunningAverageArray(Ndist,true);
        this.rightDistAvg = new RunningAverageArray(Ndist,true);
    }

    /**
     * Updates the position and orientation (yaw) of the robot based on new encoder values.
     *
     * @param leftEncoderDist  distance traveled by the left wheel (mm).
     * @param rightEncoderDist distance traveled by the right wheel (mm).
     * @param pitch   The pitch of the body connected to the wheels (in degrees), zero is up.
     * @param timeChange   The loop delta time (in seconds).
     */
    public void update(double leftEncoderDist, double rightEncoderDist, double pitch, double timeChange) {
        
        double deltaLeft, deltaRight;
        double lastLeftDistance;
        double lastRightDistance;
        double deltaDistance;
        double deltaTheta;

        // Calculate the delta pitch (degrees) of the chassis, since the last update
        // This is subtracted from the travel because it moves the encoders
        double deltaPitch = pitch - lastPitch;
        lastPitch = pitch;
        double pitchEqDist = (deltaPitch/360.0)*wheelCircumference; // Pitch Equivalent Distance

        // get the prior running average distance
        lastLeftDistance = leftDistAvg.getAverage();
        lastRightDistance = rightDistAvg.getAverage();

        // add the new distances to the running averages
        leftDistAvg.add(leftEncoderDist);
        rightDistAvg.add(rightEncoderDist);

        // get the new running average distance
        newLeftDistance = leftDistAvg.getAverage();
        newRightDistance = rightDistAvg.getAverage();
        
        // take the difference and add the pitch adjustment
        deltaLeft  = (newLeftDistance -lastLeftDistance) - pitchEqDist;
        deltaRight = (newRightDistance-lastRightDistance) - pitchEqDist;

        // Calculate the change in orientation
        deltaTheta = (deltaLeft- deltaRight) / wheelBase;
        //deltaTheta = Math.atan2((deltaLeft- deltaRight),wheelBase); // more accurate, but not much

        // Calculate the average distance traveled
        deltaDistance = (deltaLeft + deltaRight) / 2;
        s += deltaDistance;
        linearVelocity = deltaDistance/timeChange;
        veloAvg.add(linearVelocity); // add to the running average

        // Update the position and orientation
        if (deltaTheta == 0) {
            // Robot is moving straight
            x += deltaDistance * Math.cos(theta);
            y += deltaDistance * Math.sin(theta);
        } else {
            // Robot is rotating around a point
            double radius = deltaDistance / deltaTheta;
            //double radius = (deltaDistance/2) / Math.tan(deltaTheta); // more accurate, but not much
            double centerX = x - radius * Math.sin(theta);
            double centerY = y + radius * Math.cos(theta);
            theta += deltaTheta;
            noNormalTheta += deltaTheta;
            x = centerX + radius * Math.sin(theta);
            y = centerY - radius * Math.cos(theta);
        }

        // Normalize theta to the range [-pi, pi]
        theta = Angles.normalizeAngle(theta);
    }

    public double getS() {
        return s;
    }
    public double getTheta() {
        return -noNormalTheta;
    }
    public double getLinearVelocity() {
        return linearVelocity;
    }
    public double getAvgLinearVelocity() {
        return veloAvg.getAverage();
    }
}