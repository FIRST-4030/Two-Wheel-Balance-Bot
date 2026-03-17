/* This Iterative Tele OpMode is for a Two Wheel Balancing Robot With Arm
 */

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RunningAverage;
import org.firstinspires.ftc.teamcode.TwoWheelBalanceBot;

/**
 * Iterative Tele OpMode is for driving a Two Wheel Balancing Robot with Arm
 * in TANK mode.
 */
@TeleOp(name="TWB Robot Oriented drive mode")
@Disabled
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

        joystickS = new RunningAverage(7); // initialize size of running average

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
        // The left trigger is a speed booster
        joystickS.addNumber(gamepad1.left_stick_y * (1 + gamepad1.left_trigger));

        // Translate the robot
        twb.translateDrive(joystickS.getAverage(),8,7);

        // Either joystick can turn the robot.  Different speeds.
        twb.turn_teleop(gamepad1.left_stick_x,0.02);
        twb.turn_teleop(gamepad1.right_stick_x,0.01);

        twb.arm_teleop(gamepad1.right_stick_y);

        //Set arm angle straight up
        if (gamepad1.left_bumper) twb.theArm.setArmAngle(0.0);

        //Set arm angle to cargo collection
        if (gamepad1.right_trigger_pressed) twb.theArm.setArmAngle(-150.0);

        twb.claw_teleop(gamepad1.rightBumperWasPressed());

        twb.loop();  // call the MAIN CONTROL SYSTEM

        telemetry.update();
    }
}
