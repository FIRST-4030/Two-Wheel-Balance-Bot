
package org.firstinspires.ftc.teamcode.opmodes;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BlueWheelTWB;
import org.firstinspires.ftc.teamcode.Datalogger;
import org.firstinspires.ftc.teamcode.TWBMoves;
import org.firstinspires.ftc.teamcode.Term;

/**
 * This Iterative Autonomous OpMode is for a Two Wheel Balancing Robot with Arm.
 * Moves forward, moves back.
 * Used for developing motion profiles in TWBMoves.java.  Specifically the Pitch profile.
 */
@TeleOp(name="Blue Back and Forth DOE")
//@Disabled
public class Blue_back_n_forth_DOE extends OpMode {
    private BlueWheelTWB twb;
    final private TWBMoves myTWBmoves = new TWBMoves(); // used for auto
    final private ElapsedTime moveTimer = new ElapsedTime();
    private double currentPos;
    private final double DIST = 1000; // mm
    private final double DIST_TIME = 3.5; // sec
    private final double SETTLE_TIME = 5.0; // sec
    private final double ARMANGLE = -90.0;

    // Modify the Terms in init()
    private Term PITCH1;
    private Term PITCH2;
    private Term PITCH3;
    private int count = 1; // for counting the DOE

    private int NEXPERIMENTS; // total experiments, set in init

    private DatalogBnF datalogBnFDOE;  // data logger for experiments

    enum State {
        SETTLE1,
        SETTLE2,
        MOVE1,
        MOVE2,
    }

    State state = State.MOVE1;

    @Override
    public void init() {
        twb = new BlueWheelTWB(hardwareMap); // Create twb object

        // NOTE: TWO datalogs can be written!
        // Load "terms" log into a spreadsheet, filter, and sort for the lowest score.
        datalogBnFDOE = new DatalogBnF("BlueBnFterms");
        twb.writeDatalog("BlueBnFDOEfull");

        // MODIFY THESE FOR THE EXPERIMENTS.
        PITCH1 = new Term(-2.5, -1.5, 3, twb.getPos());
        PITCH2 = new Term(-2.5, -1.5, 2, twb.getPitch());
        PITCH3 = new Term(-0.5, 0.0, 2, twb.getVelocity());

        NEXPERIMENTS = PITCH1.getN() * PITCH2.getN() * PITCH3.getN();

        twb.closeClaw(); // close the claw
        twb.setArmAngle(ARMANGLE); // gets the latest state of the robot before running

        twb.start();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("BLUE TWO WHEEL BOT INIT ");
        telemetry.addLine("BACK AND FORTH MOTION PROFILING");
        telemetry.addLine("ROBOT BALANCING");
        twb.loop(this);  // call balance control system
        telemetry.update();
    }

    @Override
    public void start() {
        state = State.MOVE1;
        resetRuntime();
        moveTimer.reset();
        PITCH1.resetSum(); // holding the position sum
        PITCH2.resetSum(); // holding the velocity sum
        myTWBmoves.setPitchVector(new double[] {PITCH1.getCurrent(),PITCH2.getCurrent(),PITCH3.getCurrent()} );
    }

    @Override
    @SuppressLint("DefaultLocale")
    public void loop() {
        double[] newTargets;

        switch (state) {
            case SETTLE1: // stabilize time
                if (moveTimer.seconds() >= SETTLE_TIME) {
                    state = State.MOVE2;
                    moveTimer.reset();
                }
                break;
            case SETTLE2:
                if (moveTimer.seconds() >= SETTLE_TIME) {
                    state = State.MOVE1;
                    moveTimer.reset();
                }
                break;
            case MOVE1:
                if (moveTimer.seconds() <= DIST_TIME) {
                    newTargets = myTWBmoves.lineMove(DIST, DIST_TIME, moveTimer.seconds(), currentPos);
                    twb.setPosTarget(newTargets[0]);
                    twb.setAutoPitchTarget(newTargets[1]);
                    twb.setVeloTarget(newTargets[2]);
                    PITCH1.updateSum(twb.getPos(), newTargets[0], 0.020);
                    PITCH2.updateSum(twb.getVelocity(),newTargets[2],0.020);
                } else {
                    myTWBmoves.reverseDir = true;
                    state = State.SETTLE1;
                    moveTimer.reset();
                    currentPos = DIST; // for the next state
                    NextTest();
                }
                break;
            case MOVE2:
                if (moveTimer.seconds() <= DIST_TIME) {
                    newTargets = myTWBmoves.lineMove(DIST, DIST_TIME, moveTimer.seconds(), currentPos);
                    twb.setPosTarget(newTargets[0]);
                    twb.setAutoPitchTarget(newTargets[1]);
                    twb.setVeloTarget(newTargets[2]);
                    PITCH1.updateSum(twb.getPos(), newTargets[0], 0.020);
                    PITCH2.updateSum(twb.getVelocity(),newTargets[2],0.020);
                } else {
                    myTWBmoves.reverseDir = false;
                    state = State.SETTLE2;
                    moveTimer.reset();
                    currentPos = 0; // for the next state
                    NextTest();
                }
                break;
        }

        twb.loop(this);  // MAIN CONTROL SYSTEM

        telemetry.addLine(String.format("EXPERIMENT %d ,OF TOTAL %d", count, NEXPERIMENTS));
        telemetry.addData("State", state);
        telemetry.update();
        if (count > NEXPERIMENTS) requestOpModeStop(); // Stop the opmode

    }

    private void NextTest() {
        // datalog - one line for each experiment
        datalogBnFDOE.count.set(count);
        datalogBnFDOE.PITCH1.set(PITCH1.getCurrent());
        datalogBnFDOE.PITCH2.set(PITCH2.getCurrent());
        datalogBnFDOE.PITCH3.set(PITCH3.getCurrent());
        datalogBnFDOE.PosSum.set(PITCH1.getSum());
        datalogBnFDOE.VeloSum.set(PITCH2.getSum());
        datalogBnFDOE.score.set(PITCH1.getSum()+PITCH2.getSum());
        datalogBnFDOE.writeLine();

        // set up for the next experiment
        PITCH1.next();
        PITCH1.resetSum();
        PITCH2.resetSum();
        if (count % PITCH1.getN() == 0) {
            PITCH2.next();
        }
        if ((count % (PITCH1.getN() * PITCH2.getN())) == 0) {
            PITCH3.next();
        }

        myTWBmoves.setPitchVector(new double[] {PITCH1.getCurrent(),PITCH2.getCurrent(),PITCH3.getCurrent()} );

        count += 1;
    }

    /**
     * Datalog class encapsulates all the fields that will go into the datalog.
     */
    public static class DatalogBnF {
        // The underlying datalogger object - it cares only about an array of loggable fields
        private final Datalogger datalogger;

        // These are all of the fields that we want in the datalog.
        // Note that order here is NOT important. The order is important in the setFields() call below
        public Datalogger.GenericField count = new Datalogger.GenericField("count");
        public Datalogger.GenericField PITCH1 = new Datalogger.GenericField("PITCH1");
        public Datalogger.GenericField PITCH2 = new Datalogger.GenericField("PITCH2");
        public Datalogger.GenericField PITCH3 = new Datalogger.GenericField("PITCH3");
        public Datalogger.GenericField PosSum = new Datalogger.GenericField("Pos_Sum");
        public Datalogger.GenericField VeloSum = new Datalogger.GenericField("Velo_Sum");
        public Datalogger.GenericField score = new Datalogger.GenericField("SCORE");


        public DatalogBnF(String name) {
            // Build the underlying datalog object
            datalogger = new Datalogger.Builder()

                    // Pass through the filename
                    .setFilename(name)

                    // Request an automatic timestamp field
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)

                    // Tell it about the fields we care to log.
                    // Note that order *IS* important here! The order in which we list
                    // the fields is the order in which they will appear in the log.
                    .setFields(
                            count,
                            PITCH1,
                            PITCH2,
                            PITCH3,
                            PosSum,
                            VeloSum,
                            score
                    )
                    .build();
        }

        // Tell the datalogger to gather the values of the fields
        // and write a new line in the log.
        public void writeLine() {
            datalogger.writeLine();
        }

    }
}