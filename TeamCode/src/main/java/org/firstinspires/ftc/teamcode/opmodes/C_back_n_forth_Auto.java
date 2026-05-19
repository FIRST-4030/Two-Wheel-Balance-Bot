
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.C_TWB;
import org.firstinspires.ftc.teamcode.TWBMove;

/**
 * This Iterative Autonomous OpMode is for a Two Wheel Balancing Robot.
 * Moves forward, moves back. Does other things.
 * Used for developing motion profiles.
 */
@Autonomous(name="C Back and Forth Auto")
public class C_back_n_forth_Auto extends OpMode {
    private C_TWB twb;
    private double DIST = 1000; // mm
    private double TIME = 3.0; // sec
    private TWBMove myTWBmoves;
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
        telemetry.addLine("TWB Back-n-Forth Auto INIT ");

        if (gamepad1.dpadUpWasPressed()) DIST += 100.0;
        else if (gamepad1.dpadDownWasPressed()) DIST -= 100.0;

        if (gamepad1.dpadLeftWasPressed()) TIME += 0.25;
        else if (gamepad1.dpadRightWasPressed()) TIME -= 0.25;
        telemetry.addLine("DPAD UP - DOWN Adjusts the distance");
        telemetry.addData("Travel Distance (mm)"," %.1f", DIST);
        telemetry.addLine("DPAD LEFT - RIGHT Adjusts the time");
        telemetry.addData("Travel Time (seconds) =", TIME);

        telemetry.update();
    }

    @Override
    public void start() {
        state = State.START;
        myTWBmoves = new TWBMove(TIME,DIST);
        resetRuntime();
        moveTimer.reset();
        twb.start();
        twb.moveGearUp();
    }

    @Override
    public void loop() {
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
                    newTargets = myTWBmoves.lineMove(moveTimer.seconds(),currentPos);
                    twb.setPosTarget(newTargets[0]);
                } else if (moveTimer.seconds() <= TIME+SETTLE_TIME ) {
                    // do nothing, let settle
                } else {
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
                     newTargets = myTWBmoves.lineMove(moveTimer.seconds(), currentPos);
                     twb.setPosTarget(newTargets[0]);
                 } else if (moveTimer.seconds() <= TIME+SETTLE_TIME ) {
                     // do nothing, let settle
                 } else {
                     state = State.GEARDOWN;
                     moveTimer.reset();
                 }
                break;
            case GEARDOWN:
                if (moveTimer.seconds() <= 0.05)
                    twb.moveGearDown();
                else if (moveTimer.seconds() > 0.4)
                    requestOpModeStop();
                break;
        }

        twb.loop(this);  // MAIN CONTROL SYSTEM

        telemetry.addData("State",state);
        telemetry.update();
    }
}