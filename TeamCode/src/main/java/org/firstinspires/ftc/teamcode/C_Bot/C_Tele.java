package org.firstinspires.ftc.teamcode.C_Bot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RunningAverageArray;

/**
 * Iterative Tele OpMode for a two wheel robot
 */
@TeleOp(name="C TELEOP")
//@Disabled
public class C_Tele extends OpMode
{
    // Declare OpMode members.
    private C_TWB twb;

    /**
     * run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        twb = new C_TWB(hardwareMap); // Create twb object

        twb.init();
    }

    /**
     * run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        telemetry.addLine("INIT LOOP");
        twb.init_loop();
        twb.writeTelemetry(this);
        telemetry.update();
        twb.setFlywheel(0.0);
    }

    /**
     * run ONCE when the driver hits START
     */
    @Override
    public void start() {
        twb.start();
    }

    /**
     * run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        twb.startCycleTImer();

        if (gamepad1.rightBumperWasPressed()) twb.collectFlywheel();
        if (gamepad1.y) twb.shootFlywheel();
        if (gamepad1.leftBumperWasReleased()) twb.flywheelOff();

        // allow for variation of the max velocity
        twb.setMaxSpeedGamepad(this);

        // Translate the robot
        twb.translateDrive(gamepad1.left_stick_y);

        // Either joystick can turn the robot.  Different speeds. Sets yaw target
        twb.turn_teleop(-gamepad1.left_stick_x * 0.03);
        twb.turn_teleop(-gamepad1.right_stick_x * 0.04);


        if(gamepad1.backWasPressed()) { // toggle gear state
            if (twb.isGearDown()) twb.moveGearUp();
            else  twb.moveGearDown();
        }

        //twb.writeTelemetry(this);
        telemetry.update();

        twb.loopC(this);  // call the MAIN CONTROL SYSTEM

    }
}