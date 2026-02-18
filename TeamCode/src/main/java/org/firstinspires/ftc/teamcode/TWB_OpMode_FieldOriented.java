/* This Iterative Tele OpMode is for a Two Wheel Balancing Robot With Arm
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Iterative Tele OpMode is for a Two Wheel Balancing Robot with Arm.
 */
@TeleOp(name="TWB Tele Field Oriented")
//@Disabled
public class TWB_OpMode_FieldOriented extends OpMode
{
    // Declare OpMode members.
    private TwoWheelBalanceBot twb;
    private RunningAverage angle; // to smooth angle
    private RunningAverage speed; // to smooth aggressive joystick inputs

    /**
     * TWB Teleop Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        twb = new TwoWheelBalanceBot(hardwareMap,this); // Create twb object

        angle = new RunningAverage(10); // initialize size of running average

        speed = new RunningAverage(5); // initialize size of running average

        twb.LOG = false;
        twb.TELEMETRY = false;

        twb.init();
        /*
        The telemetry.setMsTransmissionInterval() method in the FIRST Tech Challenge (FTC) SDK controls
        how frequently telemetry data is sent from the Robot Controller to the Driver Station
        250 (milliseconds) is the default value and a good general-purpose interval that balances updates with bandwidth usage.
        100 to 50 (milliseconds) are useful for debugging or operations requiring faster updates (e.g., vision processing, rapid sensor changes).
        A lower interval provides a more real-time view of data on the Driver Station but increases communication bandwidth usage,
        which might impact other network-dependent tasks (like vision processing).
         */
        telemetry.setMsTransmissionInterval(100);
    }

    /**
     * TWB Teleop Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        telemetry.addData("TWO WHEEL BOT", "INIT LOOP");

        twb.init_loop(); // provides user a chance to change the K terms

        twb.auto_right_loop(); // gets the robot into a position to self right

        telemetry.update();
    }

    /**
     * TWB Teleop Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {

        twb.start(-90.0); // gets the latest state of the robot before running
    }

    /**
     * TWB Teleop Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        double forward = -gamepad1.left_stick_y;
        double right = -gamepad1.left_stick_x;
        double speed1 = Math.sqrt(forward*forward + right*right);
        speed.addNumber(speed1);

        // The java.lang.Math.atan2() method returns the angle (theta) in radians between the positive x-axis and a point (x, y).
        // It is a static method that takes two double arguments: y (the y-coordinate) and x (the x-coordinate).
        // It is an unknown angle if both arguments are zero

        if (speed1 <= 0.02) {
            angle.addNumber(twb.yawTarget);
        }  else  {
            angle.addNumber(Math.atan2(right, forward));

            double shortestAngleMove = twb.shortestAngleDifference(angle.getAverage(), twb.yawTarget); // radians
            telemetry.addData("shortest turn (deg)", "%.1f ", shortestAngleMove*180/Math.PI);
            twb.yawTarget += shortestAngleMove;  // Adjust yawTarget and the TWB yaw PID controller does the rest
        }

        twb.translateDrive(-speed.getAverage());

        twb.turn_teleop(0.02);

        twb.arm_teleop();

        twb.claw_teleop();

        twb.loop();  // call the MAIN CONTROL SYSTEM

        telemetry.addData("yaw Target (deg)", "%.1f ", twb.yawTarget*180/Math.PI);

        telemetry.addData("yaw Current (deg)", "%.1f ", twb.yaw*180/Math.PI);

        telemetry.addData("s position Target (mm)", "%.1f ", twb.posTarget);
        telemetry.addData("s position Odometry (mm)", "%.1f ", twb.sOdom);
        telemetry.update();
    }
}
