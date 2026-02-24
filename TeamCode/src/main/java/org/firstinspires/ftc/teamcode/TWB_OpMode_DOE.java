/* This Iterative Autonomous OpMode is for a Two Wheel Balancing Robot with Arm
 */

package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This Iterative OpMode is for a Two Wheel Balancing Robot with Arm.
 * It initializes to balancing, then runs through a series of Kposition and Kpitch values
 * with small perturbations to make the robot rock.
 * Telemetry shows the values with the lowest oscillations - write these down at the end!
 * Also, one can review the datalog and see when the robot is well balanced.
 */
//@Disabled
@TeleOp(name="TWB Design of Experiments")
public class TWB_OpMode_DOE extends OpMode {
    // Declare OpMode members.
    private TwoWheelBalanceBot twb;
    final private ElapsedTime moveTimer = new ElapsedTime();

    // DOE constants.  Modify these for the experiment
    private final double minKpos = 0.010; //
    private final double maxKpos = 0.020; //
    private final int NKPOS = 5;
    private final double minKpitch = -0.60; // -.85 for -90 to 90
    private final double maxKpitch = -0.50; // -0.70 for -90 to 90
    private final int NKPITCH = 5;
    private final double testDuration = 5.0; // seconds per experiment

    private final double JIGGLEDEG = 10; // Pitch jiggle for each experiment

    // Design of experiment variables:
    private double KPOS; // DOE value
    private double KPIT; // DOE value

    // Variables for recording position wave amplitude
    private double minPos = 1000; // mm
    private double maxPos = -1000; // mm

    // Variables for recording pitch wave amplitude
    private double minPitch = 45; // degrees
    private double maxPitch = -45; // degrees

    private int count = 1; // for counting the DOE

    private DatalogTWB datalogTWB; // datalog for full recording

    private DatalogEXP datalogEXP;  // data logger for experiments

    private double originalKpos;
    private double originalKpitch;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        twb = new TwoWheelBalanceBot(hardwareMap,this); // Create twb object

        datalogTWB = new DatalogTWB();
        datalogTWB.init("DOEfull");

        datalogEXP = new DatalogEXP("DOEexperiments");

        twb.TELEMETRY = false;
        twb.ClawIsClosed = true; // close the claw

        KPOS = minKpos;    // set the initial DOE K term
        KPIT = minKpitch;  // set the initial DOE K term

        originalKpos = twb.Kpos;     // save the original K term
        originalKpitch = twb.Kpitch; // save the original K term

        twb.init();
        twb.start(-90.0); // set the arm angle

    }

    /**
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     * The Robot MOVES (balances) on init!!!
     */
    @Override
    public void init_loop() {
        telemetry.addData("EXPERIMENT", "INIT LOOP");
        twb.loop();  // MAIN CONTROL SYSTEM

        telemetry.addData("Robot Pitch ", twb.pitch);

        telemetry.update();
    }

    /**
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        resetRuntime();
        moveTimer.reset();
    }

    /**
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    @SuppressLint("DefaultLocale")
    public void loop() {
        double incrementKpos = (maxKpos-minKpos) / NKPOS;  // calculated
        double incrementKpitch = (maxKpitch - minKpitch) / NKPITCH;   // calculated

        // give robot a jiggle at the beginning of each period to get a wave
        // while using the original K terms so that the jiggle is consistent
        if(moveTimer.seconds() < 0.1) {
            twb.autoPitchTarget = JIGGLEDEG; // add JIGGLEDEG degrees initially to jiggle
        } else if(moveTimer.seconds() <= testDuration) {
            twb.autoPitchTarget = 0.0;
            twb.Kpos = KPOS;   // set the new DOE K term
            twb.Kpitch = KPIT; // set the new DOE K term
            // during the experiment
            // build the minimum amplitude "box" on the position wave
            minPos = Math.min(twb.sOdom, minPos);
            maxPos = Math.max(twb.sOdom, maxPos);

            // build the minimum amplitude "box" on the pitch wave
            minPitch = Math.min(twb.pitch, minPitch);
            maxPitch = Math.max(twb.pitch, maxPitch);
        } else if(moveTimer.seconds() > testDuration) {
            // At the end of the experiment, only once, log data and do resets

            // datalog - one line for each experiment
            // count, KPIT, KPOS (the inputs)
            datalogEXP.count.set(count);
            datalogEXP.KPIT.set(KPIT);
            datalogEXP.KPOS.set(KPOS);
            // min, max and amplitudes (the results)
            datalogEXP.minPos.set(minPos);
            datalogEXP.maxPos.set(maxPos);
            datalogEXP.ampPos.set(maxPos - minPos);
            datalogEXP.minPitch.set(minPitch);
            datalogEXP.maxPitch.set(maxPitch);
            datalogEXP.ampPitch.set(maxPitch - minPitch);

            // The logged timestamp is taken when writeLine() is called.
            datalogEXP.writeLine();

            // set up for the next experiment
            KPIT = KPIT + incrementKpitch;
            if (count % NKPOS == 0) {
                KPIT = minKpitch; // set back to min
                KPOS = KPOS + incrementKpos;
            }

            twb.Kpos = originalKpos;   // set the original  DOE K term for the jiggle
            twb.Kpitch = originalKpitch; // set the original  DOE K term for the jiggle

            //twb.Kvelo = KPOS;  now set after the jiggle
            //twb.Kpitch = KPIT; now set after the jiggle

            moveTimer.reset();

            count += 1;

            minPos = 1000;
            maxPos = -1000;
            minPitch = 45;
            maxPitch = -45;
        }

        twb.loop();  // CALL MAIN TWB CONTROL SYSTEM

        // This datalogging should only be needed for the initial debugging of the DOE
        datalogTWB.logPosPitch(twb.sOdom, twb.posTarget, twb.pitch, twb.pitchTarget,
                twb.positionVolts,twb.pitchVolts); // for datalog every loop cycle
        datalogTWB.writeLineTWB();

        int NEXPERIMENTS = NKPOS * NKPITCH;
        telemetry.addLine(String.format("EXPERIMENT %d ,OF TOTAL %d",count, NEXPERIMENTS));
        telemetry.addData("Kposition",KPOS);
        telemetry.addData("Kpitch",KPIT);

        telemetry.update();

        if (count > NEXPERIMENTS) requestOpModeStop(); // Stop the opmode
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
        public Datalogger.GenericField KPIT = new Datalogger.GenericField("KPIT");
        public Datalogger.GenericField KPOS = new Datalogger.GenericField("KPOS");
        public Datalogger.GenericField minPos = new Datalogger.GenericField("minPos");
        public Datalogger.GenericField maxPos = new Datalogger.GenericField("maxPos");
        public Datalogger.GenericField ampPos = new Datalogger.GenericField("ampPos");

        public Datalogger.GenericField minPitch = new Datalogger.GenericField("minPitch");
        public Datalogger.GenericField maxPitch = new Datalogger.GenericField("maxPitch");
        public Datalogger.GenericField ampPitch = new Datalogger.GenericField("ampPitch");


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
                            KPIT,
                            KPOS,
                            minPos,
                            maxPos,
                            ampPos,
                            minPitch,
                            maxPitch,
                            ampPitch
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