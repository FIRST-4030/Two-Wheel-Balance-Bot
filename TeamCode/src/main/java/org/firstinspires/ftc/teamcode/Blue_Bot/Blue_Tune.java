package org.firstinspires.ftc.teamcode.Blue_Bot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RunningAverageArray;

/**
 * Iterative Tele OpMode is tuning a two wheel robot
 * Do this before running the DOE to tune
 */
@TeleOp(name="Blue TWB TUNE")
public class Blue_Tune extends OpMode
{
    // Declare OpMode members.
    private BlueWheelTWB twb;

    /**
     * run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        twb = new BlueWheelTWB(hardwareMap); // Create twb object

        twb.writeDatalog("BlueManualTune"); // needs to be part of constructor or something
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
        twb.startCycleTImer();

        tuneKterms();

        //tuneDriveTerms();

        // Translate the robot
        twb.translateDrive(gamepad1.left_stick_y);

        // turn the robot
        twb.turn_teleop(gamepad1.left_stick_x * 0.04);

        twb.arm_teleop(gamepad1.right_stick_y);

        // Set arm angle straight up. Sets arm angle target
        if (gamepad1.left_bumper) twb.setArmAngle(0.0);

        // Set arm angle to cargo collection. Sets arm angle target
        if (gamepad1.right_trigger_pressed) twb.setArmAngle(-120.0);

        twb.claw_teleop(gamepad1.rightBumperWasPressed());

        twb.loopBlue(this);  // call the MAIN CONTROL SYSTEM

        twb.writeTelemetry(this);
        telemetry.update();
    }
    /**
     * TWB method to provide buttons for tuning feedback constants.
     */
    public void tuneKterms() {
        if (gamepad1.dpadUpWasPressed()) twb.setKpos(twb.getKpos()+0.0001);
        else if (gamepad1.dpadDownWasPressed()) twb.setKpos(twb.getKpos()-0.0001);

        else if (gamepad1.dpadLeftWasPressed()) twb.setKvelo(twb.getKvelo()+0.00002);
        else if (gamepad1.dpadRightWasPressed()) twb.setKvelo(twb.getKvelo()-0.00002);

        else if (gamepad1.xWasPressed()) twb.setKpitch(twb.getKpitch()+0.002);
        else if (gamepad1.bWasPressed()) twb.setKpitch(twb.getKpitch()-0.002);

        else if (gamepad1.yWasPressed()) twb.setKpitchRate(twb.getKpitchRate()+0.0002);
        else if (gamepad1.aWasPressed()) twb.setKpitchRate(twb.getKpitchRate()-0.0002);

        telemetry.addLine(" --- ");
        telemetry.addData("K pos   DPAD +UP -DOWN", "%.5f",twb.getKpos());
        telemetry.addData("K velo  DPAD +LEFT -RIGHT", "%.6f",twb.getKvelo());
        telemetry.addLine(" --- ");
        telemetry.addData("K pitch +X -B","%.5f", twb.getKpitch());
        telemetry.addData("K pitch rate +Y -A", "%.6f", twb.getKpitchRate());
    }
    /**
     * TWB method to provide buttons for tuning drive terms.
     */
    public void tuneDriveTerms() {
        //if (gamepad1.dpadUpWasPressed()) twb.MMPLoop+=0.5;
        //else if (gamepad1.dpadDownWasPressed()) twb.MMPLoop-=0.5;

        //else if (gamepad1.dpadLeftWasPressed()) twb.DEGPLoop+=0.5;
        //else if (gamepad1.dpadRightWasPressed()) twb.DEGPLoop-=0.5;

        telemetry.addData("MM/Loop   DPAD +UP -DOWN", twb.getMaxAllowedVelocity());
    }
}
