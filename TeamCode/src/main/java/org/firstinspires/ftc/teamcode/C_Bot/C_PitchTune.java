
package org.firstinspires.ftc.teamcode.C_Bot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RunningAverageArray;

import java.util.Locale;

/**
 * This Iterative Design of Experiments OpMode is for a Two Wheel Balancing Robot.
 *  Use it to find the pitch angle when the robot is at a zero position target.
 */
@TeleOp(name="C Pitch Zero Tuner")
//@Disabled
public class C_PitchTune extends OpMode {
    // Declare OpMode members.
    private C_TWB twb;

    private RunningAverageArray robotPos; // to provide steady position telemetry in init

    private double pitchFuzz = 0.0;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        twb = new C_TWB(hardwareMap); // Create twb object

        twb.writeDatalog("C_PitchZero"); // default log

        robotPos = new RunningAverageArray(150,true); // for robot position telemetry

        twb.init();
    }

    /**
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     * The Robot MOVES (balances) on init!!!
     */
    @Override
    public void init_loop() {
        twb.init_loop();
        telemetry.addLine("Tune Zero-Pitch-Target so average position is zero");
        telemetry.addLine(" ---");
        //telemetry.addLine("Tune Vertical-Center-of-Mass so Std Dev is near zero");
        //telemetry.addLine(" ---");
        telemetry.addLine("WAIT THREE SECONDS BETWEEN CHANGES FOR NUMBERS TO STABILIZE");

        twb.writeTelemetry(this);

        telemetry.update();
    }

    /**
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {

        twb.start();
        pitchFuzz = twb.getZeroPitchTarget();
    }

    /**
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        twb.startCycleTImer();

        if (gamepad1.dpadUpWasPressed()) pitchFuzz += 0.1;
        else if (gamepad1.dpadDownWasPressed()) pitchFuzz -= 0.1;
        twb.setZeroPitchTarget(pitchFuzz);

//        if (gamepad1.dpadLeftWasPressed()) twb.setVerticalCM(twb.getVerticalCM()+1.0);
//        else if (gamepad1.dpadRightWasPressed()) twb.setVerticalCM(twb.getVerticalCM()-1.0);

        if(gamepad1.backWasPressed()) { // toggle gear state
            if (twb.isGearDown()) twb.moveGearUp();
            else  twb.moveGearDown();
        }

        robotPos.add(twb.getPos()); // add to running average, for telemetry only

        telemetry.addLine(String.format(Locale.US, "s Position Target %.0f ,Current %.0f (mm)",
                twb.getPosTarget(),twb.getPos()));
        telemetry.addLine(" ---");
        telemetry.addData("AVERAGE Position (mm)","  %.0f", robotPos.getAverage());
        telemetry.addData("DPAD UP+ DOWN- Zero-Pitch-Target Adjust (deg)"," %.1f", pitchFuzz);
        telemetry.addLine(" ---");
//        telemetry.addData("Position Standard Deviation (mm)","  %.1f", robotPos.getStandardDeviation());
//        telemetry.addData("Robot Vertical Center of Mass (mm)"," %.1f", twb.getVerticalCM());
//        telemetry.addLine("DPAD LEFT+ RIGHT-  VertCM Adjust");
//        telemetry.addLine(" ---");

        telemetry.update();

        twb.loopC(this);  // call balance control system

    }

}