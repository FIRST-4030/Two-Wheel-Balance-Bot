/* This Iterative Tele OpMode is for a Two Wheel Balancing Robot With Arm
 */

package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Iterative Teleoperated OpMode is for a Two Wheel Balancing Robot with Arm,
 * Using a Field Oriented "swerve drive" type control
 */
@TeleOp(name="TWB Field Oriented drive mode")
//@Disabled
public class TWB_OpMode_FieldOriented extends OpMode
{
    // Declare OpMode members.
    private TwoWheelBalanceBot twb;
    private RunningAverage speed; // to smooth aggressive joystick inputs
    double yaw1 = 0.0; // single value derived from joystick angle
    private double shortestYawMove = 0.0;
    private static double MAXDELTARAD = 0.08; // YAW Maximum Delta Radians update per loop
    private DatalogFO datalog; // create the data logger object for Field Oriented

    /**
     * TWB Teleop Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        twb = new TwoWheelBalanceBot(hardwareMap,this); // Create twb object

        speed = new RunningAverage(5); // initialize size of running average

        twb.LOG = false;
        twb.TELEMETRY = false;

        datalog = new DatalogFO("FieldOrientlog");

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
        telemetry.addData("FIELD ORIENTED", "DRIVE WITH LEFT JOYSTICK");
        telemetry.addData("TO RESET ORIENTATION", "PRESS Y");

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
    @SuppressLint("DefaultLocale")
    public void loop() {

        double forward = -gamepad1.left_stick_y;
        double right = -gamepad1.left_stick_x;
        double speed1 = Math.sqrt(forward*forward + right*right);
        speed.addNumber(speed1);

        // The java.lang.Math.atan2() method returns the angle (theta) in radians between the positive x-axis and a point (x, y).
        // It is a static method that takes two double arguments: y (the y-coordinate) and x (the x-coordinate).
        // It is an unknown angle if both arguments are zero, thus check for some joystick input (speed1)

        if (speed1 > 0.02) {
            yaw1 = Math.atan2(right, forward);
            shortestYawMove = shortestAngleDifference(yaw1, twb.yawTarget); // radians

            // Adjust yawTarget, with constraint on how much, and the TWB yaw PID controller does the rest
            if (shortestYawMove > MAXDELTARAD) {
                twb.yawTarget += MAXDELTARAD;
            } else if (shortestYawMove < -MAXDELTARAD) {
                twb.yawTarget -= MAXDELTARAD;
            } else {
                twb.yawTarget += shortestYawMove;
            }
        }

        twb.translateDrive(-speed.getAverage());

        twb.turn_teleop(gamepad1.right_stick_x,0.02);

        twb.arm_teleop();

        twb.claw_teleop();

        if (gamepad1.yWasPressed()) {
            twb.imuYawReset();
        }

        twb.loop();  // call the MAIN CONTROL SYSTEM

        telemetry.addLine(String.format("Yaw Target %.1f ,Current %.1f (degrees)",twb.yawTarget*180/Math.PI,twb.yaw*180/Math.PI));
        telemetry.addLine(String.format("s Position Target %.1f ,Current %.1f (mm)",twb.posTarget,twb.sOdom));
        telemetry.update();

//        datalog.yaw.set(twb.yaw*180/Math.PI);
//        datalog.yawTarget.set(twb.yawTarget*180/Math.PI);
//        datalog.shortestYawMove.set(shortestYawMove *180/Math.PI);
//        datalog.speed.set(speed.getAverage());
//        // The logged timestamp is taken when writeLine() is called.
//        datalog.writeLine();
    }
    /**
     * DatalogFO class encapsulates all the fields that will go into the datalog.
     */
    public static class DatalogFO {
        // The underlying datalogger object - it cares only about an array of loggable fields
        private final Datalogger datalogger;

        // These are all of the fields that we want in the datalog.
        // Note that order here is NOT important. The order is important in the setFields() call below
        public Datalogger.GenericField yaw = new Datalogger.GenericField("yaw");
        public Datalogger.GenericField yawTarget = new Datalogger.GenericField("yawTarget");
        public Datalogger.GenericField shortestYawMove = new Datalogger.GenericField("shortYawMv");
        public Datalogger.GenericField speed = new Datalogger.GenericField("speed");

        public DatalogFO(String name) {
            // Build the underlying datalog object
            datalogger = new Datalogger.Builder()

                    // Pass through the filename
                    .setFilename(name)

                    // Request an automatic timestamp field
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)

                    // Tell it about the fields we care to log.
                    // Note that order *IS* important here! The order in which we list
                    // the fields is the order in which they will appear in the log.
                    .setFields(
                            yaw,
                            yawTarget,
                            shortestYawMove,
                            speed
                    )
                    .build();
        }

        // Tell the datalogger to gather the values of the fields
        // and write a new line in the log.
        public void writeLine() {
            datalogger.writeLine();
        }
    }
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
