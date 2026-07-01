
package org.firstinspires.ftc.teamcode.opmodes;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.C_TWB;
import org.firstinspires.ftc.teamcode.Datalogger;
import org.firstinspires.ftc.teamcode.Term;

import java.util.Locale;

/**
 * This Iterative Design of Experiments OpMode is for a Two Wheel Balancing Robot.
 * It drives the robot back and forth through a series of tests.
 *  The tests vary these members:
 *  One datalog records a summary, the other records all data.
 */
@TeleOp(name="C BACK N FORTH Design of Experiments")
//@Disabled
public class C_BackNForth_DOE extends OpMode {
    // Declare OpMode members.
    private C_TWB twb;
    final private ElapsedTime moveTimer = new ElapsedTime();

    // DOE constants.  Modify these for the experiment
    private final double testDuration = 4.0; // seconds per experiment
    private final double DISTANCE = 300.0; // Travel distance for each test (mm)
    private boolean forward = true; // to keep track of direction
    private double virtualJoystick = 0.0;

    // Modify the Terms in init()
    private Term mmPerLoop;
    private Term degPerLoop;
    //private Term UNUSED1;
    //private Term UNUSED2;

    // Internal variables
    private int count = 1; // for counting the DOE
    private int NEXPERIMENTS; // total experiments, set in init

    private DatalogEXP datalogEXP;  // data logger for experiments

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        twb = new C_TWB(hardwareMap); // Create twb object

        // NOTE: TWO datalogs can be written!
        // Load "terms" log into a spreadsheet, filter, and sort for the lowest score.
        datalogEXP = new DatalogEXP("C_DOE_bnf");

        twb.writeLog("C_DOE_bnf_Full"); // This log will be bigger

        // MODIFY THESE FOR THE EXPERIMENTS.
        mmPerLoop = new Term(3.0,4.0,5,twb.getKpos());
        //UNUSED1 = new Term(-0.0022,-0.0022,1,twb.getKvelo());
        degPerLoop = new Term(-1.0,0.0,5,twb.getKpitch());
        //UNUSED2 = new Term(-0.0044,-0.0043,1,twb.getKpitchRate());
        NEXPERIMENTS = mmPerLoop.getN() * degPerLoop.getN();

        twb.init();
    }

    /**
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     * The Robot MOVES (balances) on init!!!
     */
    @Override
    public void init_loop() {
        telemetry.addLine("DOE to determine MM_Per_Loop and DEG_Per_Loop");
        telemetry.addLine(String.format(Locale.US, "TOTAL EXPERIMENTS %d",NEXPERIMENTS));
        telemetry.addLine(String.format(Locale.US, "TOTAL TIME %.2f",NEXPERIMENTS*testDuration));

        twb.init_loop();

        telemetry.update();
    }

    /**
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        twb.start();
        resetRuntime();
        moveTimer.reset();
    }

    /**
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    @SuppressLint("DefaultLocale")
    public void loop() {
        if (moveTimer.seconds() < 0.03) {
            // set the new DOE K terms
            twb.setMMPLoop(mmPerLoop.getCurrent());
            twb.setDEGPLoop(degPerLoop.getCurrent());

            if (forward) {
                virtualJoystick = -1.0;
            } else {
                virtualJoystick = 1.0;
            }

        } else if (moveTimer.seconds() <= testDuration) {

            double thisPos = twb.getPos();
            double thisPitch = twb.getPitch();
            double thisDT = twb.getDeltaTime();

            // build the minimum amplitude "box" on the position wave
            mmPerLoop.updateMinMax(thisPos);

            // build the minimum amplitude "box" on the pitch wave
            degPerLoop.updateMinMax(thisPitch);

            // Integrate the position and pitch errors over time
            mmPerLoop.updateSum(thisPos, twb.getPosTarget(), thisDT);
            degPerLoop.updateSum(thisPitch, twb.getPitchTarget(), thisDT);

            // turn off the virtual joystick when distance is reached
            if (forward && twb.getPos() >= DISTANCE) virtualJoystick = 0.0;
            if (!forward && twb.getPos() <= 0.0) virtualJoystick = 0.0;

        } else if(moveTimer.seconds() > testDuration ) {
            virtualJoystick = 0.0;
            // At the end of the experiment, only once, log data and do resets

            // datalog - one line for each experiment
            // count, KPIT, KPOS (the inputs)
            datalogEXP.count.set(count);
            datalogEXP.MMPerLoop.set(mmPerLoop.getCurrent());
            datalogEXP.DEGPerLoop.set(degPerLoop.getCurrent());
            // min, max and amplitudes (the results)
//            datalogEXP.minPos.set(mmPerLoop.getMin());
//            datalogEXP.maxPos.set(mmPerLoop.getMax());
//            double ampPos = mmPerLoop.getMax() - mmPerLoop.getMin();
//            datalogEXP.ampPos.set(ampPos);
//            double AvgPos = (mmPerLoop.getMin() + mmPerLoop.getMax())/2.0;
//            datalogEXP.AvgPos.set(AvgPos);
            datalogEXP.PosError.set(mmPerLoop.getSum());
//            datalogEXP.minPitch.set(degPerLoop.getMin());
//            datalogEXP.maxPitch.set(degPerLoop.getMax());
//            double ampPitch = degPerLoop.getMax() - degPerLoop.getMin();
//            datalogEXP.ampPitch.set(ampPitch);
            datalogEXP.PitchError.set(degPerLoop.getSum());
            //datalogEXP.score.set(ampPitch*4.0+ampPos+Math.abs(AvgPos)); // low score wins!
            //datalogEXP.score.set(16.0* degPerLoop.getSum() + mmPerLoop.getSum()); // low score wins!

            // The logged timestamp is taken when writeLine() is called.
            datalogEXP.writeLine();

            // set up for the next experiment
            mmPerLoop.next();
            if(count % mmPerLoop.getN() == 0) {
                degPerLoop.next();
            }

            moveTimer.reset();
            forward = !forward;

            count += 1;

            mmPerLoop.resetMinMax();
            degPerLoop.resetMinMax();
            mmPerLoop.resetSum();
            degPerLoop.resetSum();
        }
        // Translate the robot by setting position, velocity and pitch targets
        twb.translateDrive(virtualJoystick,mmPerLoop.getCurrent(),degPerLoop.getCurrent());

        twb.loopC(this);  // CALL MAIN TWB CONTROL SYSTEM

        telemetry.addLine(String.format("EXPERIMENT %d  OF TOTAL %d",count, NEXPERIMENTS));
        telemetry.addLine(" --- ");

        telemetry.addData("mmPerLoop","%.2f", mmPerLoop.getCurrent());
        //telemetry.addData("UNUSED","%.7f", UNUSED1.getCurrent());
        telemetry.addLine(" --- ");

        telemetry.addData("degPerLoop","%.2f", degPerLoop.getCurrent());
        //telemetry.addData("KpitchRate","%.7f", UNUSED2.getCurrent());

        telemetry.update();

        if (count > NEXPERIMENTS) {
            twb.moveGearDown();
            if(moveTimer.seconds() > 0.4) requestOpModeStop(); // Stop the opmode
        }
    }
    /**
     * Datalog class encapsulates all the fields that will go into the datalog.
     */
    public static class DatalogEXP {
        // The underlying datalogger object - it cares only about an array of loggable fields
        private final Datalogger datalogger;

        // These are all of the fields that we want in the datalog.
        // Note that order here is NOT important. The order is important in the setFields() call below
        public Datalogger.GenericField count = new Datalogger.GenericField("count");
        public Datalogger.GenericField MMPerLoop = new Datalogger.GenericField("MMPerLoop");
        public Datalogger.GenericField DEGPerLoop = new Datalogger.GenericField("DEGPerLoop");

        public Datalogger.GenericField minPos = new Datalogger.GenericField("minPos");
        public Datalogger.GenericField maxPos = new Datalogger.GenericField("maxPos");
        public Datalogger.GenericField ampPos = new Datalogger.GenericField("ampPos");
        public Datalogger.GenericField AvgPos = new Datalogger.GenericField("AVG_Pos");
        public Datalogger.GenericField PosError = new Datalogger.GenericField("Pos_Error");

        public Datalogger.GenericField minPitch = new Datalogger.GenericField("minPitch");
        public Datalogger.GenericField maxPitch = new Datalogger.GenericField("maxPitch");
        public Datalogger.GenericField ampPitch = new Datalogger.GenericField("ampPitch");
        public Datalogger.GenericField PitchError = new Datalogger.GenericField("Pitch_Error");

        public Datalogger.GenericField score = new Datalogger.GenericField("SCORE");


        public DatalogEXP(String name) {
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
                            MMPerLoop,
                            DEGPerLoop,
                            minPos,
                            maxPos,
                            ampPos,
                            AvgPos,
                            PosError,
                            minPitch,
                            maxPitch,
                            ampPitch,
                            PitchError,
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