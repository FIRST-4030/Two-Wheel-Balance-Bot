/* This Iterative Autonomous OpMode is for a Two Wheel Balancing Robot with Arm
 */

package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This Iterative Design of Experiments OpMode is for a Two Wheel Balancing Robot with Arm.
 * It initializes to balancing, then runs through a series of Kposition and Kpitch terms
 * with a perturbation (jiggle) to make the robot rock.
 * A datalog records the min/max of position and pitch for each test, with the
 * expectation that the lowest mix/max are the most stable terms.
 */
//@Disabled
@TeleOp(name="TWB Design of Experiments")
public class TWB_OpMode_DOE extends OpMode {
    // Declare OpMode members.
    private TwoWheelBalanceBot twb;
    final private ElapsedTime moveTimer = new ElapsedTime();

    // DOE constants.  Modify these for the experiment
    private final double ARMANGLE = -90.0;
    private final double testDuration = 5.0; // seconds per experiment
    private final double JIGGLEDEG = 10; // Pitch jiggle for each experiment

    // Modify the Terms in init()
    private Term Kpos;
    private Term Kpitch;
    private Term Kvelo;
    private Term KpitchRate;

    // Internal variables
    private int count = 1; // for counting the DOE
    private int NEXPERIMENTS; // total experiments, set in init
    private DatalogTWB datalogTWB; // datalog for full recording

    private DatalogEXP datalogEXP;  // data logger for experiments

    private RunningAverage robotPos; // to look for the balanced angle

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        twb = new TwoWheelBalanceBot(hardwareMap,this); // Create twb object

        // NOTE: TWO datalogs are written!
        datalogTWB = new DatalogTWB();
        datalogTWB.init("DOEfull");

        datalogEXP = new DatalogEXP("DOEexperiments");

        twb.TELEMETRY = false;
        twb.ClawIsClosed = true; // close the claw

        // MODIFY THESE FOR THE EXPERIMENTS
        Kpos = new Term(0.014,0.018,3,twb.Kpos);
        Kpitch = new Term(-0.60,-0.54,3,twb.Kpitch);
        Kvelo = new Term(0.014,0.018,3,twb.Kvelo);  // 0.020 breaks bot
        KpitchRate = new Term(-0.026,-0.020,3,twb.KpitchRate);

        NEXPERIMENTS = Kpos.getN() * Kpitch.getN() * Kvelo.getN() * KpitchRate.getN();

        robotPos = new RunningAverage(100); // initialize size of running average

        twb.init();
        twb.start(ARMANGLE); // set the arm angle
    }

    /**
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     * The Robot MOVES (balances) on init!!!
     */
    @Override
    public void init_loop() {
        telemetry.addData("DOE EXPERIMENT, ARM Angle (deg) =", ARMANGLE);
        twb.loop();  // MAIN CONTROL SYSTEM

        robotPos.addNumber(twb.sOdom);
        telemetry.addData("Robot Position (mm) (Averaged)","  %.1f", robotPos.getAverage());

        telemetry.addData("Robot Pitch (deg)"," %.1f", twb.pitch);

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
        // give robot a jiggle at the beginning of each period to get a wave
        // while using the original K terms so that the jiggle is consistent
        if(moveTimer.seconds() < 0.1) {
            twb.Kpos = Kpos.orig;
            twb.Kpitch = Kpitch.orig;
            twb.Kvelo = Kvelo.orig;
            twb.KpitchRate = KpitchRate.orig;
            twb.autoPitchTarget = JIGGLEDEG; // add JIGGLEDEG degrees initially to jiggle

        } else if(moveTimer.seconds() <= testDuration) {
            twb.autoPitchTarget = 0.0;
            // set the new DOE K terms
            twb.Kpos = Kpos.current;
            twb.Kpitch = Kpitch.current;
            twb.Kvelo = Kvelo.current;
            twb.KpitchRate = KpitchRate.current;

            // during the experiment, after the jiggle, record min/max
            if(moveTimer.seconds() > 0.3) {
                // build the minimum amplitude "box" on the position wave
                Kpos.updateMinMax(twb.sOdom);

                // build the minimum amplitude "box" on the pitch wave
                Kpitch.updateMinMax(twb.pitch);
            }

        } else if(moveTimer.seconds() > testDuration) {
            // At the end of the experiment, only once, log data and do resets

            // datalog - one line for each experiment
            // count, KPIT, KPOS (the inputs)
            datalogEXP.count.set(count);
            datalogEXP.KPIT.set(Kpitch.current);
            datalogEXP.KPOS.set(Kpos.current);
            datalogEXP.KVELO.set(Kvelo.current);
            datalogEXP.KPITRATE.set(KpitchRate.current);
            // min, max and amplitudes (the results)
            datalogEXP.minPos.set(Kpos.min);
            datalogEXP.maxPos.set(Kpos.max);
            double ampPos = Kpos.max - Kpos.min;
            datalogEXP.ampPos.set(ampPos);
            double AvgPos = (Kpos.min+Kpos.max)/2.0;
            datalogEXP.AvgPos.set(AvgPos);
            datalogEXP.minPitch.set(Kpitch.min);
            datalogEXP.maxPitch.set(Kpitch.max);
            double ampPitch = Kpitch.max - Kpitch.min;
            datalogEXP.ampPitch.set(ampPitch);
            datalogEXP.score.set(ampPitch*4.0+ampPos+Math.abs(AvgPos)); // low score wins!

            // The logged timestamp is taken when writeLine() is called.
            datalogEXP.writeLine();

            // set up for the next experiment
            KpitchRate.next();
            if(count % Kvelo.getN() == 0) {
                Kvelo.next();
            }
            if((count % (Kvelo.getN()*KpitchRate.getN())) == 0) {
                Kpos.next();
            }
            if((count % (Kvelo.getN()*KpitchRate.getN()*Kpos.getN())) == 0) {
                Kpitch.next();
            }

            moveTimer.reset();

            count += 1;

            Kpos.resetMinMax();
            Kpitch.resetMinMax();
        }

        twb.loop();  // CALL MAIN TWB CONTROL SYSTEM

        // This datalogging should only be needed for the initial debugging of the DOE
        datalogTWB.logPosPitch(twb.sOdom, twb.posTarget, twb.pitch, twb.pitchTarget,
                twb.positionVolts,twb.pitchVolts); // for datalog every loop cycle
        datalogTWB.writeLineTWB();

        telemetry.addLine(String.format("EXPERIMENT %d ,OF TOTAL %d",count, NEXPERIMENTS));
        telemetry.addData("Kposition",Kpos.current);
        telemetry.addData("Kpitch",Kpitch.current);
        telemetry.addData("Kvelo",Kvelo.current);
        telemetry.addData("KpitchRate",KpitchRate.current);

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
        public Datalogger.GenericField KVELO = new Datalogger.GenericField("KVELO");
        public Datalogger.GenericField KPITRATE = new Datalogger.GenericField("KPITRATE");

        public Datalogger.GenericField minPos = new Datalogger.GenericField("minPos");
        public Datalogger.GenericField maxPos = new Datalogger.GenericField("maxPos");
        public Datalogger.GenericField ampPos = new Datalogger.GenericField("ampPos");
        public Datalogger.GenericField AvgPos = new Datalogger.GenericField("AVG_Pos");

        public Datalogger.GenericField minPitch = new Datalogger.GenericField("minPitch");
        public Datalogger.GenericField maxPitch = new Datalogger.GenericField("maxPitch");
        public Datalogger.GenericField ampPitch = new Datalogger.GenericField("ampPitch");
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
                            KPIT,
                            KPOS,
                            KVELO,
                            KPITRATE,
                            minPos,
                            maxPos,
                            ampPos,
                            AvgPos,
                            minPitch,
                            maxPitch,
                            ampPitch,
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