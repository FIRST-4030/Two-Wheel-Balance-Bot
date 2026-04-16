package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BlueWheelTWB;
import org.firstinspires.ftc.teamcode.C_TWB;
import org.firstinspires.ftc.teamcode.RunningAverageArray;

/**
 * Iterative Tele OpMode is tuning a two wheel robot
 * Do this before running the DOE to tune
 */
@TeleOp(name="C TWB TUNE")
//@Disabled
public class C_Tune extends OpMode
{
    // Declare OpMode members.
    private C_TWB twb;

    private RunningAverageArray joystickS; // to smooth aggressive joystick inputs


    /**
     * run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        twb = new C_TWB(hardwareMap); // Create twb object

        twb.setDriveMotors(true,false,false);

        twb.writeDatalog("CManualTune");

        joystickS = new RunningAverageArray(12,false); // initialize size of running average

        twb.init();
    }

    /**
     * run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        telemetry.addLine("TWO Wheel Balance Robot Manual Tuner");

        telemetry.update();
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

        tuneKterms();

        //tuneDriveTerms();

        // Use running average of the joystick to smooth aggressive inputs.
        // The left trigger is a speed booster
        joystickS.add(gamepad1.left_stick_y * (1 + gamepad1.left_trigger));

        // Translate the robot by setting position, velocity and pitch targets
        twb.translateDrive(joystickS.getAverage(),twb.MMPLoop,twb.DEGPLoop);

        // Either joystick can turn the robot.  Different speeds. Sets yaw target
        twb.turn_teleop(gamepad1.left_stick_x * 0.03);
        twb.turn_teleop(gamepad1.right_stick_x * 0.04);

        twb.loop(this);  // call the MAIN CONTROL SYSTEM

        if(gamepad1.start) {
            twb.stop(this);
        }

        twb.writeTelemetry(this);
        telemetry.update();
    }
    @Override
    public void stop() {
        twb.stop(this);
    }
    /**
     * TWB method to provide buttons for tuning feedback constants.
     */
    public void tuneKterms() {
        if (gamepad1.dpadUpWasPressed()) twb.setKpos(twb.getKpos()+0.001);
        else if (gamepad1.dpadDownWasPressed()) twb.setKpos(twb.getKpos()-0.001);

        else if (gamepad1.dpadLeftWasPressed()) twb.setKvelo(twb.getKvelo()+0.0001);
        else if (gamepad1.dpadRightWasPressed()) twb.setKvelo(twb.getKvelo()-0.0001);

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
