
package org.firstinspires.ftc.teamcode.C_Bot;

import static org.firstinspires.ftc.teamcode.C_Bot.C_TWB.GEARDOWNTIME;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MoveProfiles;

/**
 * This Iterative Autonomous OpMode is for a Two Wheel Balancing Robot.
 * Moves forward, moves back. Does other things.
 * Used for developing motion profiles.
 */
@Autonomous(name="C Auto 1")
public class C_Auto1 extends OpMode {
    private C_TWB twb;
    private double DIST = 700; // mm
    private double TIME = 2.0; // sec
    private MoveProfiles myTWBmoves;
    final private ElapsedTime moveTimer = new ElapsedTime();
    private double currentPos;

    enum State {
        START,
        MOVE1,
        SHOOT,
        MOVE2,
        GEARDOWN
    }
    State state = State.START;

    @Override
    public void init() {
        twb = new C_TWB(hardwareMap); // Create twb object

        twb.writeDatalog("CLogAutoBnF");

        twb.moveGearDown();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Set a Distance for robot to move back and forth ");
        telemetry.addLine(" --- ");

        double maxVelo = twb.getMaxAllowedVelocity();
        double maxAccel = twb.getMaxAllowedAccel();

        double accelDist = Math.PI*(maxVelo*maxVelo)/(2*maxAccel);  // Dist based on Max Accel

        if (gamepad1.dpadUpWasPressed() && (DIST < accelDist)) DIST += 100.0;
        else if (gamepad1.dpadDownWasPressed() && (DIST > 0)) DIST -= 100.0;

        TIME = 2.0 * Math.sqrt((Math.PI*DIST/2)/maxAccel);

        if (gamepad1.dpadLeftWasPressed() && (maxAccel > 10)) twb.setMaxAllowedAccel(maxAccel-100.0);
        else if (gamepad1.dpadRightWasPressed() && (maxAccel < 3000.0)) twb.setMaxAllowedAccel(maxAccel + 100.0);

        maxAccel = twb.getMaxAllowedAccel();  // update again

        telemetry.addLine("DPAD UP - DOWN Adjusts the distance");
        telemetry.addData("Travel Distance (mm)"," %.0f", DIST);
        telemetry.addData("Distance Limit based on Accel"," %.0f", accelDist);
        telemetry.addLine(" --- ");
        telemetry.addData("Travel Time (seconds) =", TIME);
        telemetry.addLine(" --- ");
        telemetry.addData("Max allowed Velocity (mm/sec)"," %.1f", maxVelo);
        telemetry.addData("Max Acceleration (mm/sec)"," %.1f", maxAccel);
        telemetry.addLine("DPAD LEFT - RIGHT Adjusts the Acceleration");
        telemetry.update();
    }

    @Override
    public void start() {
        state = State.START;
        myTWBmoves = new MoveProfiles(TIME,DIST);
        resetRuntime();
        moveTimer.reset();
        twb.start();
        twb.moveGearUp();
    }

    @Override
    public void loop() {
        twb.startCycleTImer();

        double[] newTargets;

        double SETTLE_TIME = 2.0; // seconds

        switch (state) {
            case START: // stabilize time
                if (getRuntime() >= SETTLE_TIME) {
                    state = State.MOVE1;
                    moveTimer.reset();
                    currentPos = 0; // force current pos to zero, for offset in next state
                }
                break;
            case MOVE1:
                if (moveTimer.seconds() <= TIME ) {
                    newTargets = myTWBmoves.lineMoveLoop(moveTimer.seconds(),currentPos);
                    twb.setPosTarget(newTargets[0]);
                } else if (moveTimer.seconds() > TIME+SETTLE_TIME ) {
                    state = State.SHOOT;
                    moveTimer.reset();
                    currentPos = DIST; // for the next state
                }
                break;
            case SHOOT:
                if (moveTimer.seconds() <= 0.05)
                    twb.shootFlywheel();
                else if (moveTimer.seconds() > 1.0) {
                    twb.flywheelOff();
                    state = State.MOVE2;
                    moveTimer.reset();
                }

                break;
            case MOVE2:
                 if (moveTimer.seconds() <= TIME) {
                     myTWBmoves.reverseDir = true;
                     newTargets = myTWBmoves.lineMoveLoop(moveTimer.seconds(), currentPos);
                     twb.setPosTarget(newTargets[0]);
                 } else if (moveTimer.seconds() > TIME+SETTLE_TIME ) {
                     state = State.GEARDOWN;
                     moveTimer.reset();
                 }
                break;
            case GEARDOWN:
                if (moveTimer.seconds() <= 0.03)
                    twb.moveGearDown();
                else if (moveTimer.seconds() > GEARDOWNTIME)
                    requestOpModeStop();
                break;
        }

        telemetry.addData("State",state);
        telemetry.update();

        twb.loopC(this);  // MAIN CONTROL SYSTEM

    }
}