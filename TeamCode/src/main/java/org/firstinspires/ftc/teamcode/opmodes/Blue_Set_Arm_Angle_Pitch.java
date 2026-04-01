
package org.firstinspires.ftc.teamcode.opmodes;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BlueWheelTWB;
import org.firstinspires.ftc.teamcode.DatalogTWB;
import org.firstinspires.ftc.teamcode.Datalogger;
import org.firstinspires.ftc.teamcode.RunningAverageArray;
import org.firstinspires.ftc.teamcode.Term;

/**
 * This OpMode allows one to tune the zero pitch values for different arm angles.
 *  Adjust the pitch and look for when the robots position is zero.
 */
@TeleOp(name="Blue Set Arm Angle Pitch")
@Disabled
public class Blue_Set_Arm_Angle_Pitch extends OpMode {
    // Declare OpMode members.
    private BlueWheelTWB twb;

    private double ARMANGLE = -20.0;
    private RunningAverageArray robotPos; // to provide steady position telemetry in init
    private double pitchFuzz = 0.8;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        twb = new BlueWheelTWB(hardwareMap); // Create twb object

        twb.closeClaw(); // close the claw

        robotPos = new RunningAverageArray(150,true); // for robot position telemetry

        twb.setArmAngle(ARMANGLE); // gets the latest state of the robot before running

        twb.start();
    }

    /**
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     * The Robot MOVES (balances) on init!!!
     */
    @Override
    public void init_loop() {
        if (gamepad1.dpadUpWasPressed()) pitchFuzz += 0.1;
        else if (gamepad1.dpadDownWasPressed()) pitchFuzz -= 0.1;
        twb.setAutoPitchTarget(pitchFuzz);

        if (gamepad1.dpadLeftWasPressed()) ARMANGLE += 20.0;
        else if (gamepad1.dpadRightWasPressed()) ARMANGLE -= 20.0;
        twb.setArmAngle(ARMANGLE); // gets the latest state of the robot before running

        telemetry.addLine("DPAD UP - DOWN Adjusts the Pitch Fuzz");
        telemetry.addData("Robot Pitch (deg)"," %.1f", twb.getPitch());
        telemetry.addData("Pitch  FUZZ (deg)"," %.1f", pitchFuzz);
        telemetry.addLine("DPAD LEFT - RIGHT Adjusts the Arm Angle");
        telemetry.addData("ARM Angle (deg) =", ARMANGLE);

        twb.loop(this);  // call balance control system

        telemetry.addLine("ADJUST FUZZ SO THAT POSITION IS NEAR ZERO");

        robotPos.add(twb.getPos()); // for telemetry only
        telemetry.addData("Robot Position (mm) (Averaged)","  %.1f", robotPos.getAverage());

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
    @SuppressLint("DefaultLocale")
    public void loop() {

        twb.loop(this);  // CALL MAIN TWB CONTROL SYSTEM

        telemetry.addData("Pitch  FUZZ (deg)"," %.1f", pitchFuzz);

        robotPos.add(twb.getPos()); // for telemetry only
        telemetry.addData("Robot Position (mm) (Averaged)","  %.1f", robotPos.getAverage());

        twb.arm_teleop(gamepad1.right_stick_y);

        twb.writeTelemetry(this);

        telemetry.update();
    }
}