
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BlueWheelTWB;
import org.firstinspires.ftc.teamcode.TWBMove;

/**
 * This Iterative Autonomous OpMode is for a Two Wheel Balancing Robot with Arm.
 * Moves forward, moves back.
 * Used for developing motion profiles.
 */
@Autonomous(name="Blue Back and Forth")
//@Disabled
public class Blue_back_n_forth extends OpMode {
    private BlueWheelTWB twb;
    private double DIST = 1000; // mm
    private double TIME = 3.0; // sec
    private TWBMove myTWBmoves;
    final private ElapsedTime moveTimer = new ElapsedTime();
    private double currentPos;

    private final double SETTLE_TIME = 3.0; // sec

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

        twb.closeClaw();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("BLUE TWB Back-n-Forth Auto INIT ");

        if (gamepad1.dpadUpWasPressed()) DIST += 100.0;
        else if (gamepad1.dpadDownWasPressed()) DIST -= 100.0;

        if (gamepad1.dpadLeftWasPressed()) TIME += 0.25;
        else if (gamepad1.dpadRightWasPressed()) TIME -= 0.25;
        telemetry.addLine("DPAD UP - DOWN Adjusts the distance");
        telemetry.addData("Travel Distance (mm)"," %.1f", DIST);
        telemetry.addLine("DPAD LEFT - RIGHT Adjusts the time");
        telemetry.addData("Travel Time (seconds) =", TIME);

        twb.auto_right_loop(); // gets the robot into a position to self right
        twb.closeClaw();

        telemetry.update();
    }

    @Override
    public void start() {
        state = State.START;
        myTWBmoves = new TWBMove(TIME,DIST);
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
                if (getRuntime() <= (SETTLE_TIME + TIME) ) {
                    newTargets = myTWBmoves.lineMove(moveTimer.seconds(),currentPos);
                    twb.setPosTarget(newTargets[0]);
                    //twb.setAutoPitchTarget(newTargets[1]);
                    //twb.setVeloTarget(newTargets[2]);
                } else {
                    state = State.MOVE2;
                    moveTimer.reset();
                    currentPos = DIST; // for the next state
                }
                break;
            case MOVE2:
                 if (getRuntime() <= (SETTLE_TIME + TIME + SETTLE_TIME)) {
                     // Sit for a bit to let robot stabilize after opening claw
                     moveTimer.reset();

                 } else if (getRuntime() <= (2*SETTLE_TIME + 2* TIME)) {
                     twb.openClaw();

                     myTWBmoves.reverseDir = true;
                     newTargets = myTWBmoves.lineMove(moveTimer.seconds(), currentPos);
                     twb.setPosTarget(newTargets[0]);
                     //twb.setAutoPitchTarget(newTargets[1]);
                     //twb.setVeloTarget(newTargets[2]);
                 } else {
                     state = State.DONE;
                     moveTimer.reset();
                 }
                break;
            case DONE:
                if (getRuntime() > (3*SETTLE_TIME + 2* TIME + 1) ) requestOpModeStop();
                break;
        }

        twb.loop(this);  // MAIN CONTROL SYSTEM

        telemetry.addData("State",state);
        telemetry.update();
    }
}