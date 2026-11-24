package org.firstinspires.ftc.teamcode.Blue_Bot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RunningAverageArray;

/**
 * Iterative Tele OpMode is for driving a Two Wheel Balancing Robot with Arm
 * in regular (robot oriented) gamepad mode.
 */
@TeleOp(name="Blue TWB Tele")
public class Blue_Tele extends OpMode
{
    // Declare OpMode members.
    private BlueWheelTWB twb;

    /**
     * run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        twb = new BlueWheelTWB(hardwareMap); // Create twb object

        twb.writeDatalog("BlueTele");

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

        twb.startCycleTImer();

        // Translate the robot
        twb.translateDrive(gamepad1.left_stick_y);

        // turn the robot.
        twb.turn_teleop(gamepad1.left_stick_x * 0.04);

        twb.arm_teleop(gamepad1.right_stick_y);

        if (gamepad1.left_bumper) twb.clawWave();

        // Set arm angle to cargo collection. Sets arm angle target
        if (gamepad1.right_trigger_pressed) twb.setArmAngle(-120.0);

        twb.claw_teleop(gamepad1.rightBumperWasPressed());

        twb.loopBlue(this);  // call the MAIN CONTROL SYSTEM

        twb.writeTelemetry(this);
        telemetry.update();
    }
}
