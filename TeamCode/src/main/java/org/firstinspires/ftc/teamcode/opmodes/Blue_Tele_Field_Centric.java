package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.Angles.shortestAngleDifference;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BlueWheelTWB;
import org.firstinspires.ftc.teamcode.RunningAverage;

/**
 * Iterative Tele OpMode is for driving a Two Wheel Balancing Robot with Arm
 * in Field Centric Mode mode.
 */
@TeleOp(name="Blue TWB Tele Field Centric")
//@Disabled
public class Blue_Tele_Field_Centric extends OpMode
{
    // Declare OpMode members.
    private BlueWheelTWB twb;
    private RunningAverage speed; // to smooth aggressive joystick inputs

    /**
     * run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        twb = new BlueWheelTWB(hardwareMap); // Create twb object

        speed = new RunningAverage(6); // initialize size of running average
        /*
        The telemetry.setMsTransmissionInterval() method in the FIRST Tech Challenge SDK controls
        how frequently telemetry data is sent from the Robot Controller to the Driver Station
        250 (milliseconds) is the default value and a good general-purpose interval.
        100 to 50 (milliseconds) are useful for debugging or operations requiring faster updates.
        A lower interval provides a more real-time view of data on the Driver Station but
        increases communication bandwidth usage,
         */
        telemetry.setMsTransmissionInterval(150);
    }

    /**
     * run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        telemetry.addLine("BLUE TWO WHEEL BOT INIT ");
        telemetry.addLine("FIELD ORIENTED --- DRIVE WITH LEFT JOYSTICK");

        telemetry.addLine("ROBOT MOVING INTO POSITION TO START");
        twb.auto_right_loop(); // gets the robot into a position to self right

        telemetry.addLine("TO RESET ORIENTATION PRESS THE Y");
        telemetry.update();
    }

    /**
     * run ONCE when the driver hits START
     */
    @Override
    public void start() {
        twb.start();
        twb.setArmAngle(-90.0); // gets the latest state of the robot before running
    }

    /**
     * run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    @SuppressLint("DefaultLocale")
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double right = -gamepad1.left_stick_x;
        double speed1 = Math.sqrt(forward*forward + right*right);
        speed.addNumber(speed1);

        // The java.lang.Math.atan2() method returns the angle (theta) in radians between
        // the positive x-axis and a point (x, y).
        // It takes two double arguments: y (the y-coordinate) and x (the x-coordinate).
        // It is an unknown angle if both arguments are zero, thus check for some joystick input

        if (speed1 > 0.02) {
            double yaw1 = Math.atan2(right, forward);
            double shortestYawMove = shortestAngleDifference(yaw1, twb.getYawTarget()); // radians

            // Adjust yawTarget, with constraint on how much, and the TWB yaw PID controller does the rest
            // YAW Maximum Delta Radians update per loop
            double MAXDELTARAD = 0.08;
            if (shortestYawMove > MAXDELTARAD) {
                twb.setYawTarget(twb.getYawTarget()+ MAXDELTARAD);
            } else if (shortestYawMove < -MAXDELTARAD) {
                twb.setYawTarget(twb.getYawTarget()- MAXDELTARAD);
            } else {
                twb.setYawTarget(twb.getYawTarget()+shortestYawMove);
            }
        }

        twb.translateDrive(-speed.getAverage(),8,7);

        twb.turn_teleop(gamepad1.right_stick_x * 0.04); // also can turn this way

        twb.arm_teleop(gamepad1.right_stick_y);

        //Set arm angle straight up
        if (gamepad1.left_bumper) twb.setArmAngle(0.0);

        //Set arm angle to cargo collection
        if (gamepad1.right_trigger_pressed) twb.setArmAngle(-120.0);

        twb.claw_teleop(gamepad1.rightBumperWasPressed());

        if (gamepad1.yWasPressed()) {
            twb.imuReset();
            // NOT WORKING IF YAW ANGLE IS GREATER THAN 180 OR -180
        }

        twb.loop(this);  // call the MAIN CONTROL SYSTEM

        telemetry.addLine(String.format("Yaw Target %.1f ,Current %.1f (degrees)",twb.getYawTarget()*180/Math.PI,twb.getYaw()*180/Math.PI));

        twb.writeTelemetry(this);

        telemetry.update();
    }
}
