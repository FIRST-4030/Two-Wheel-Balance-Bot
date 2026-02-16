/* This Iterative Tele OpMode is for a Two Wheel Balancing Robot With Arm
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Iterative Tele OpMode is for a Two Wheel Balancing Robot with Arm.
 */
@TeleOp(name="TWB Tele Tank Drive")
//@Disabled
public class TWB_OpMode_TankDrive extends OpMode
{
    // Declare OpMode members.
    private TwoWheelBalanceBot twb;

    private RunningAverage joystickS; // to smooth aggressive joystick inputs

    /**
     * TWB Teleop Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        twb = new TwoWheelBalanceBot(hardwareMap,this); // Create twb object

        joystickS = new RunningAverage(5); // initialize size of running average

        twb.LOG = false;

        twb.init();

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

        // get running average of the joystick to smooth aggressive inputs
        joystickS.addNumber(gamepad1.left_stick_y);

        // get teleoperated inputs
        twb.translateDrive(joystickS.getAverage());

        twb.turn_teleop(0.02);

        twb.arm_teleop();

        twb.claw_teleop();

        twb.loop();  // call the MAIN CONTROL SYSTEM

        telemetry.update();
    }
}
