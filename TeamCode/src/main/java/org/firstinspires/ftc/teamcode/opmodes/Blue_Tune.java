package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BlueWheelTWB;
import org.firstinspires.ftc.teamcode.RunningAverageArray;

/**
 * Iterative Tele OpMode is tuning a two wheel robot
 * Do this before running the DOE to tune
 */
@TeleOp(name="Blue TWB TUNE")
//@Disabled
public class Blue_Tune extends OpMode
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

        //twb.writeDatalog("BlueManualTune"); // needs to be part of constructor or something

        joystickS = new RunningAverageArray(12,false); // initialize size of running average
    }

    /**
     * run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        telemetry.addLine("TWO Wheel Balance Robot Manual Tuner");

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

        //tuneKterms();

        tuneDriveTerms();

        // Use running average of the joystick to smooth aggressive inputs.
        // The left trigger is a speed booster
        joystickS.add(gamepad1.left_stick_y * (1 + gamepad1.left_trigger));

        // Translate the robot by setting position, velocity and pitch targets
        twb.translateDrive(joystickS.getAverage(),twb.MMPLoop,twb.DEGPLoop);

        // Either joystick can turn the robot.  Different speeds. Sets yaw target
        twb.turn_teleop(gamepad1.left_stick_x * 0.03);
        twb.turn_teleop(gamepad1.right_stick_x * 0.04);

        twb.arm_teleop(gamepad1.right_stick_y);

        // Set arm angle straight up. Sets arm angle target
        if (gamepad1.left_bumper) twb.setArmAngle(0.0);

        // Set arm angle to cargo collection. Sets arm angle target
        if (gamepad1.right_trigger_pressed) twb.setArmAngle(-120.0);

        twb.claw_teleop(gamepad1.rightBumperWasPressed());

        twb.loop(this);  // call the MAIN CONTROL SYSTEM

        twb.writeTelemetry(this);
        telemetry.update();
    }
    /**
     * TWB method to provide buttons for tuning feedback constants.
     */
    public void tuneKterms() {
        if (gamepad1.dpadUpWasPressed()) twb.setKpos(twb.getKpos()+0.001);
        else if (gamepad1.dpadDownWasPressed()) twb.setKpos(twb.getKpos()-0.001);

        else if (gamepad1.dpadLeftWasPressed()) twb.setKvelo(twb.getKvelo()+0.001);
        else if (gamepad1.dpadRightWasPressed()) twb.setKvelo(twb.getKvelo()-0.001);

        else if (gamepad1.xWasPressed()) twb.setKpitch(twb.getKpitch()+0.01);
        else if (gamepad1.bWasPressed()) twb.setKpitch(twb.getKpitch()-0.01);

        else if (gamepad1.yWasPressed()) twb.setKpitchRate(twb.getKpitchRate()+0.001);
        else if (gamepad1.aWasPressed()) twb.setKpitchRate(twb.getKpitchRate()-0.001);


        telemetry.addData("K pos   DPAD +UP -DOWN", twb.getKpos());
        telemetry.addData("K velo  DPAD +LEFT -RIGHT", twb.getKvelo());
        telemetry.addData("K pitch +X -B", twb.getKpitch());
        telemetry.addData("K pitch rate +Y -A", twb.getKpitchRate());
    }
    /**
     * TWB method to provide buttons for tuning drive terms.
     */
    public void tuneDriveTerms() {
        if (gamepad1.dpadUpWasPressed()) twb.MMPLoop+=0.5;
        else if (gamepad1.dpadDownWasPressed()) twb.MMPLoop-=0.5;

        else if (gamepad1.dpadLeftWasPressed()) twb.DEGPLoop+=0.5;
        else if (gamepad1.dpadRightWasPressed()) twb.DEGPLoop-=0.5;

        telemetry.addData("MM/Loop   DPAD +UP -DOWN", twb.MMPLoop);
        telemetry.addData("DEG/Loop  DPAD +LEFT -RIGHT", twb.DEGPLoop);
    }
}
