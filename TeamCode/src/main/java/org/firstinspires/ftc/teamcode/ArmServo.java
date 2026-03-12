package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Object to control a (arm) rotation, driven by a servo.
 * Assumes a linear relation between servo value and angle.
 * Implements a rotational velocity control on the servo.
  */
public class ArmServo {
    private final Servo armServo;
    private double currentPos = 0; // current position (degrees)
    private double targetPos = 0; // target position (degrees)
    private double posMax; // maximum position to prevent damage (degrees)
    private double posMin; // minimum position to prevent damage (degrees)
    private final double maxVelocity; // maximum deg/sec allowed (always positive, work either direction)

    // CONSTANTS TO CONVERT LOCAL ARM ANGLES to SERVO VALUES
    private final double m; // this is the slope (m) of the servoPosition vs angle line
    private final double b; // this is the intercept (b) of the servoPosition vs angle line

    /**
     * TWBArmServo constructor.
     * @param hardwareMap FTC hardware map pointer
     * @param servoName the name of the servo in the config file
     * @param servoVal1 servo value at first rig point
     * @param ang1 angle at first servo value
     * @param servoVal2 servo value at second rig point
     * @param ang2 angle at second servo value
     * @param maxV maximum servo angular velocity in degrees/sec
      */
    public ArmServo(HardwareMap hardwareMap, String servoName, double servoVal1,
                    double ang1, double servoVal2, double ang2, double maxV) {

        armServo = hardwareMap.get(Servo.class, servoName);

        m = (servoVal2-servoVal1)/(ang2-ang1); // slope of the servoPosition vs angle line
        b = servoVal1 - (m*ang1);  // intercept of the servoPosition vs angle line

        maxVelocity = maxV;
    }

    /**
     * setLimits method
     * @param min minimum servo angle in degrees
     * @param max maximum servo angle in degrees
     */
    public void setLimits(double min, double max) {
        posMax = max;
        posMin = min;
    }
    /**
     * setArmAngle method.  Updates the target arm position.
     * @param armAngle new arm angle Target
     */
    public void setArmAngle(double armAngle) {targetPos = armAngle;}

    /**
     * updateArm method. Sets the servo value. Call this continuously to move the arm.
     * @param deltaTime This is the loop time and typically around 0.02 seconds
     */
    public void updateArm(double deltaTime){
        // Update the arm smoothly
        updatePosition(deltaTime);

        // convert the arm angle into a servo value
        double servoTarget = m*currentPos + b;

        armServo.setPosition(servoTarget); // this moves the servo
    }

    /**
     * updatePosition method. Internal method to provide smooth arm position update.
     * @param deltaTime loop time in seconds
     */
    private void updatePosition(double deltaTime) {
        double deltaPos = targetPos-currentPos; // position change being asked for
        double deltaPosMax = maxVelocity * deltaTime; // convert rate limit to delta position based on loop time
        // apply velocity (rate) limit
        if (Math.abs(deltaPos) > deltaPosMax) deltaPos = Math.signum(deltaPos) * deltaPosMax;

        currentPos += deltaPos; // update current position

        // check limits
        if (currentPos > posMax) currentPos = posMax;
        else if (currentPos < posMin) currentPos = posMin;
    }

    /**
     * getAngle method.  Returns the current position of the arm.
     * @return currentPos degrees
     */
    public double getAngle() { return currentPos;}

    /**
     * getTargetAngle method. Returns the target position of the arm.
     * @return targetPos degrees
     */
    public double getTargetAngle() { return targetPos; }
}
