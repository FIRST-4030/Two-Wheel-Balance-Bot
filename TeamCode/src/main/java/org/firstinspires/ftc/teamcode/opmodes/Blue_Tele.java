package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BlueWheelTWB;
import org.firstinspires.ftc.teamcode.RunningAverageArray;

/**
 * Iterative Tele OpMode is for driving a Two Wheel Balancing Robot with Arm
 * in regular (robot oriented) gamepad mode.
 */
@TeleOp(name="Blue TWB Tele")
//@Disabled
public class Blue_Tele extends OpMode
{
    // Declare OpMode members.
    private BlueWheelTWB twb;

    private RunningAverageArray joystickS; // to smooth aggressive joystick inputs

    /**
     * run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        twb = new BlueWheelTWB(hardwareMap); // Create twb object

        //twb.writeDatalog("BlueTele");

        joystickS = new RunningAverageArray(12,false); // initialize size of running average
        /*
        The telemetry.setMsTransmissionInterval() method in the FIRST Tech Challenge SDK controls
        how frequently telemetry data is sent from the Robot Controller to the Driver Station
        250 (milliseconds) is the default value and a good general-purpose interval.
        100 to 50 (milliseconds) are useful for debugging or operations requiring faster updates.
        A lower interval provides a more real-time view of data on the Driver Station but
        increases communication bandwidth usage,
         */
        telemetry.setMsTransmissionInterval(250);
    }

    /**
     * run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        telemetry.addData("BLUE TWO WHEEL BOT", "INIT LOOP");

        telemetry.addData("AUTO-RIGHT", "ACTIVE");
        twb.auto_right_loop(); // gets the robot into a position to self right

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
    public void loop() {

        // Use running average of the joystick to smooth aggressive inputs.
        // The left trigger is a speed booster
        joystickS.add(gamepad1.left_stick_y * (1 + gamepad1.left_trigger));

        // Translate the robot by setting position, velocity and pitch targets
        twb.translateDrive(joystickS.getAverage(),twb.MMPLoop,twb.DEGPLoop);

        // Either joystick can turn the robot.  Different speeds. Sets yaw target
        twb.turn_teleop(gamepad1.left_stick_x * 0.03);
        twb.turn_teleop(gamepad1.right_stick_x * 0.04);

        twb.arm_teleop(gamepad1.right_stick_y);

        if (gamepad1.left_bumper) twb.clawWave();

        // Set arm angle to cargo collection. Sets arm angle target
        if (gamepad1.right_trigger_pressed) twb.setArmAngle(-120.0);

        twb.claw_teleop(gamepad1.rightBumperWasPressed());

        twb.loop(this);  // call the MAIN CONTROL SYSTEM

        twb.writeTelemetry(this);
        telemetry.update();
    }
}
