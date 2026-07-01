
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.C_TWB;
import org.firstinspires.ftc.teamcode.RunningAverageArray;

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

        twb.writeLog("C_PitchZero"); // This log will be bigger

        robotPos = new RunningAverageArray(250,true); // for robot position telemetry

        twb.init();
    }

    /**
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     * The Robot MOVES (balances) on init!!!
     */
    @Override
    public void init_loop() {
        twb.init_loop();
        telemetry.addLine("Opmode to tune Pitch-Zero: so position is zero");
        telemetry.addLine(" and to tune Vertical Center of Mass: to minimize oscillation:");
        telemetry.update();
    }

    /**
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        twb.start();
    }

    /**
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed()) pitchFuzz += 0.1;
        else if (gamepad1.dpadDownWasPressed()) pitchFuzz -= 0.1;
        twb.setAutoPitchTarget(pitchFuzz);

        if (gamepad1.dpadLeftWasPressed()) twb.setVerticalCM(twb.getVerticalCM()+1.0);
        else if (gamepad1.dpadRightWasPressed()) twb.setVerticalCM(twb.getVerticalCM()-1.0);

        if(gamepad1.backWasPressed()) { // toggle gear state
            if (twb.isGearDown()) twb.moveGearUp();
            else  twb.moveGearDown();
        }

        twb.loopC(this);  // call balance control system

        robotPos.add(twb.getPos()); // for telemetry only
        telemetry.addData("Robot Position (mm) (Averaged)","  %.1f", robotPos.getAverage());

        telemetry.addData("Robot Pitch TARGET (deg)"," %.1f", twb.getPitchTarget());
        telemetry.addData("DPAD UP+ DOWN- Pitch Adjust (deg)"," %.1f", pitchFuzz);
        telemetry.addLine(" ---");
        telemetry.addData("Robot Vertical Center of Mass (mm)"," %.1f", twb.getVerticalCM());
        telemetry.addLine("DPAD LEFT+ RIGHT-  VertCM Adjust");
        telemetry.addLine(" ---");
        twb.writeTelemetry(this);

        telemetry.update();
    }

}