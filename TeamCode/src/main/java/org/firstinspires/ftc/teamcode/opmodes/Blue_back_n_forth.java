
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BlueWheelTWB;
import org.firstinspires.ftc.teamcode.TWBMoves;

/**
 * This Iterative Autonomous OpMode is for a Two Wheel Balancing Robot with Arm.
 * Moves forward, moves back.
 * Used for developing motion profiles.
 */
@Autonomous(name="Blue TWB Back and Forth")
//@Disabled
public class Blue_back_n_forth extends OpMode {
    private BlueWheelTWB twb;
    final private TWBMoves myTWBmoves = new TWBMoves(); // used for auto
    final private ElapsedTime moveTimer = new ElapsedTime();
    private double currentPos;
    private final double DIST = 1000; // mm
    private final double DIST_TIME = 2.8; // sec
    private final double SETTLE_TIME = 1.0; // sec

    enum State {
        START,
        MOVE1,
        MOVE2,
        DONE
    }
    State state = State.START;

    @Override
    public void init() {
        twb = new BlueWheelTWB(hardwareMap); // Create twb object

        twb.writeDatalog("BlueLogAutoBnF");

        twb.closeClaw(); // close the claw
    }

    @Override
    public void init_loop() {
        telemetry.addLine("BLUE TWO WHEEL BOT INIT ");
        telemetry.addLine("BACK AND FORTH MOTION PROFILING");

        telemetry.addLine("ROBOT MOVING INTO POSITION TO START");

        twb.auto_right_loop(); // gets the robot into a position to self right

        telemetry.update();
    }

    @Override
    public void start() {
        state = State.START;
        resetRuntime();
        moveTimer.reset();
        twb.start();
        twb.setArmAngle(-90.0); // gets the latest state of the robot before running
    }

    @Override
    public void loop() {
        double[] newTargets;

        switch (state) {
            case START: // stabilize time
                if (getRuntime() >= SETTLE_TIME) {
                    state = State.MOVE1;
                    moveTimer.reset();
                    currentPos = 0; // force current pos to zero, for offset in next state
                }
                break;
            case MOVE1:
                if (getRuntime() <= (SETTLE_TIME +DIST_TIME) ) {
                    newTargets = myTWBmoves.lineMove(DIST,DIST_TIME, moveTimer.seconds(),currentPos);
                    twb.setPosTarget(newTargets[0]);
                    twb.setAutoPitchTarget(newTargets[1]);
                    twb.setVeloTarget(newTargets[2]);
                } else {
                    state = State.MOVE2;
                    moveTimer.reset();
                    currentPos = DIST; // for the next state
                }
                break;
            case MOVE2:
                 if (getRuntime() <= (SETTLE_TIME +DIST_TIME+ SETTLE_TIME)) {
                     // Sit for a bit to let robot stabilize after opening claw
                     moveTimer.reset();
                 } else if (getRuntime() <= (2*SETTLE_TIME + 2*DIST_TIME)) {
                         myTWBmoves.reverseDir = true;
                         newTargets = myTWBmoves.lineMove(DIST,DIST_TIME, moveTimer.seconds(),currentPos);
                         twb.setPosTarget(newTargets[0]);
                         twb.setAutoPitchTarget(newTargets[1]);
                         twb.setVeloTarget(newTargets[2]);
                 } else {
                         state = State.DONE;
                         moveTimer.reset();
                 }
                break;
            case DONE:
                if (getRuntime() > (2*SETTLE_TIME + 2*DIST_TIME + 1) ) requestOpModeStop();
                break;
        }

        twb.loop(this);  // MAIN CONTROL SYSTEM

        telemetry.addData("State",state);
        telemetry.update();
    }
}