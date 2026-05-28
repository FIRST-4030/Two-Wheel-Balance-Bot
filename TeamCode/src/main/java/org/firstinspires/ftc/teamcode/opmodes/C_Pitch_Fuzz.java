
package org.firstinspires.ftc.teamcode.opmodes;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.C_TWB;
import org.firstinspires.ftc.teamcode.Datalogger;
import org.firstinspires.ftc.teamcode.RunningAverageArray;
import org.firstinspires.ftc.teamcode.Term;

/**
 * This Iterative Design of Experiments OpMode is for a Two Wheel Balancing Robot.
 *  Use it to find the pitch angle when the robot is at the position target.
 */
@TeleOp(name="C Pitch Zero Finder")
//@Disabled
public class C_Pitch_Fuzz extends OpMode {
    // Declare OpMode members.
    private C_TWB twb;
    final private ElapsedTime moveTimer = new ElapsedTime();

    private RunningAverageArray robotPos; // to provide steady position telemetry in init

    private double pitchFuzz = 0.0;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        twb = new C_TWB(hardwareMap); // Create twb object

        twb.writeDatalog("C_PitchZero"); // This log will be bigger

        robotPos = new RunningAverageArray(200,true); // for robot position telemetry

        twb.init();
    }

    /**
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     * The Robot MOVES (balances) on init!!!
     */
    @Override
    public void init_loop() {
        twb.init_loop();
        twb.writeTelemetry(this);
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

        twb.loop(this);  // call balance control system

        robotPos.add(twb.getPos()); // for telemetry only
        telemetry.addData("Robot Position (mm) (Averaged)","  %.1f", robotPos.getAverage());

        telemetry.addData("Robot Pitch TARGET (deg)"," %.1f", twb.getPitchTarget());
        telemetry.addData("DPAD UP+ DOWN- Pitch  FUZZ (deg)"," %.1f", pitchFuzz);
        telemetry.addLine(" ---");

        twb.writeTelemetry(this);

        telemetry.update();
    }

}