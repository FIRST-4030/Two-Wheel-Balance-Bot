package org.firstinspires.ftc.teamcode.C_Bot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RunningAverageArray;

/**
 * Iterative Tele OpMode for tuning a two wheel robot
 * Do this before running the DOE to tune
 */
@TeleOp(name="C TWB TUNE")
//@Disabled
public class C_Tune extends OpMode
{
    // Declare OpMode members.
    private C_TWB twb;

    /**
     * run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        twb = new C_TWB(hardwareMap); // Create twb object

        twb.writeDatalog("CManualTune");

        twb.init();
    }

    /**
     * run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        telemetry.addLine("INIT LOOP");
        // allow for variation of the max velocity
        twb.setMaxSpeedGamepad(this);
        //tuneKterms();
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
        //if (gamepad1.y) twb.shootFlywheel();
        if (gamepad1.leftBumperWasReleased()) twb.flywheelOff();

        tuneKterms();

        //tuneDriveTerms();

        // Translate the robot by setting position, velocity and pitch targets
        twb.translateDrive(gamepad1.left_stick_y);

        // Either joystick can turn the robot.  Different speeds. Sets yaw target
        twb.turn_teleop(-gamepad1.left_stick_x * 0.03);
        twb.turn_teleop(-gamepad1.right_stick_x * 0.04);

        if(gamepad1.backWasPressed()) { // toggle gear state
            if (twb.isGearDown()) twb.moveGearUp();
            else  twb.moveGearDown();
        }

        //twb.writeTelemetry(this);
        //telemetry.update();

        twb.loopC(this);  // call the MAIN CONTROL SYSTEM

    }

    /**
     * TWB method to provide buttons for tuning feedback constants.
     */
    public void tuneKterms() {
        if (gamepad1.dpadUpWasPressed()) twb.setKpos(twb.getKpos()+0.0001);
        else if (gamepad1.dpadDownWasPressed()) twb.setKpos(twb.getKpos()-0.0001);

        else if (gamepad1.dpadLeftWasPressed()) twb.setKvelo(twb.getKvelo()+0.00001);
        else if (gamepad1.dpadRightWasPressed()) twb.setKvelo(twb.getKvelo()-0.00001);

        else if (gamepad1.yWasPressed()) twb.setKpitch(twb.getKpitch()+0.002);
        else if (gamepad1.aWasPressed()) twb.setKpitch(twb.getKpitch()-0.002);

        else if (gamepad1.bWasPressed()) twb.setKpitchRate(twb.getKpitchRate()+0.0002);
        else if (gamepad1.xWasPressed()) twb.setKpitchRate(twb.getKpitchRate()-0.0002);

        telemetry.addLine(" --- ");
        telemetry.addData("K pos   DPAD +UP -DOWN", twb.getKpos());
        telemetry.addData("K velo  DPAD +LEFT -RIGHT", "%.6f",twb.getKvelo());
        telemetry.addLine(" --- ");
        telemetry.addData("K pitch +Y -A", twb.getKpitch());
        telemetry.addData("K pitch rate +X -B", "%.6f",twb.getKpitchRate());
        telemetry.addLine(" --- ");

    }

}
