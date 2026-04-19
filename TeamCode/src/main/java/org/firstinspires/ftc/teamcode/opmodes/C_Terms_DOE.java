
package org.firstinspires.ftc.teamcode.opmodes;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BlueWheelTWB;
import org.firstinspires.ftc.teamcode.C_TWB;
import org.firstinspires.ftc.teamcode.Datalogger;
import org.firstinspires.ftc.teamcode.RunningAverageArray;
import org.firstinspires.ftc.teamcode.Term;

/**
 * This Iterative Design of Experiments OpMode is for a Two Wheel Balancing Robot with Arm.
 * It initializes to balancing, then runs through a series of Kposition and Kpitch terms
 * with a perturbation (jiggle) to make the robot rock.
 * A datalog records the min/max of position and pitch for each test, with the
 * expectation that the lowest mix/max are the most stable terms.
 */
@TeleOp(name="C Bot Terms Design of Experiments")
//@Disabled
public class C_Terms_DOE extends OpMode {
    // Declare OpMode members.
    private C_TWB twb;
    final private ElapsedTime moveTimer = new ElapsedTime();

    // DOE constants.  Modify these for the experiment
    private final double testDuration = 3.0; // seconds per experiment
    private final double JIGGLEDEG = 2.0; // Pitch jiggle for each experiment

    // Modify the Terms in init()
    private Term Kpos;
    private Term Kpitch;
    private Term Kvelo;
    private Term KpitchRate;

    // Internal variables
    private int count = 1; // for counting the DOE
    private int NEXPERIMENTS; // total experiments, set in init

    private DatalogEXP datalogEXP;  // data logger for experiments

    private RunningAverageArray robotPos; // to provide steady position telemetry in init

    private double pitchFuzz = 0.0;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        twb = new C_TWB(hardwareMap); // Create twb object

        // NOTE: TWO datalogs can be written!
        // Load "terms" log into a spreadsheet, filter, and sort for the lowest score.
        datalogEXP = new DatalogEXP("C_DOEterms");

        twb.writeDatalog("C_DOEFull"); // This log will be bigger

        // MODIFY THESE FOR THE EXPERIMENTS. KPOS CHANGES WITH ARM ANGLE
//        Kpos = new Term(0.017,0.021,3,twb.getKpos());
//        Kvelo = new Term(0.015,0.019,3,twb.getKvelo());  // 0.020 breaks bot
//        Kpitch = new Term(-0.61,-0.57,3,twb.getKpitch());
//        KpitchRate = new Term(-0.028,-0.022,3,twb.getKpitchRate());
        Kpos = new Term(0.00360,0.0040,4,twb.getKpos());
        Kvelo = new Term(0.00170,0.00200,4,twb.getKvelo());
        Kpitch = new Term(-0.0615,-0.0605,2,twb.getKpitch());
        KpitchRate = new Term(-0.00515,-0.00505,2,twb.getKpitchRate());
        NEXPERIMENTS = Kpos.getN() * Kpitch.getN() * Kvelo.getN() * KpitchRate.getN();

        robotPos = new RunningAverageArray(100,true); // for robot position telemetry

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

        telemetry.addLine("DOE to determine Kpos, Kvelo, Kpitch & KpitchRate");
        telemetry.addLine(" ---");

        twb.loop(this);  // call balance control system

        robotPos.add(twb.getPos()); // for telemetry only
        telemetry.addData("Robot Position (mm) (Averaged)","  %.1f", robotPos.getAverage());

        telemetry.addData("Robot Pitch (deg)"," %.1f", twb.getPitch());
        telemetry.addData("DPAD UP+ DOWN- Pitch  FUZZ (deg)"," %.1f", pitchFuzz);

        telemetry.update();
    }

    /**
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        //twb.start();
        resetRuntime();
        moveTimer.reset();
        //Kpos.setTargetValue(0.0); // target position of the robot is zero
        //Kpitch.setTargetValue(twb.getPitchTarget());
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
            twb.setKpos(Kpos.getOriginal());
            twb.setKpitch(Kpitch.getOriginal());
            twb.setKvelo(Kvelo.getOriginal());
            twb.setKpitchRate(KpitchRate.getOriginal());

            twb.setAutoPitchTarget(JIGGLEDEG+pitchFuzz); // add JIGGLEDEG degrees initially to jiggle

        } else if(moveTimer.seconds() <= testDuration) {
            // set the new DOE K terms
            twb.setKpos(Kpos.getCurrent());
            twb.setKpitch(Kpitch.getCurrent());
            twb.setKvelo(Kvelo.getCurrent());
            twb.setKpitchRate(KpitchRate.getCurrent());

            twb.setAutoPitchTarget(pitchFuzz);

            // during the experiment, after the jiggle, record min/max
            if(moveTimer.seconds() > 0.2) {
                double thisPos = twb.getPos();
                double thisPitch = twb.getPitch();
                double thisDT = twb.getDeltaTime();

                // build the minimum amplitude "box" on the position wave
                Kpos.updateMinMax(thisPos);

                // build the minimum amplitude "box" on the pitch wave
                Kpitch.updateMinMax(thisPitch);

                // Integrate the position and pitch errors over time
                Kpos.updateSum(thisPos, 0.0,thisDT);
                Kpitch.updateSum(thisPitch, twb.getPitchTarget(),thisDT);
            }

        } else if(moveTimer.seconds() > testDuration) {
            // At the end of the experiment, only once, log data and do resets

            // datalog - one line for each experiment
            // count, KPIT, KPOS (the inputs)
            datalogEXP.count.set(count);
            datalogEXP.KPIT.set(Kpitch.getCurrent());
            datalogEXP.KPOS.set(Kpos.getCurrent());
            datalogEXP.KVELO.set(Kvelo.getCurrent());
            datalogEXP.KPITRATE.set(KpitchRate.getCurrent());
            // min, max and amplitudes (the results)
            datalogEXP.minPos.set(Kpos.getMin());
            datalogEXP.maxPos.set(Kpos.getMax());
            double ampPos = Kpos.getMax() - Kpos.getMin();
            datalogEXP.ampPos.set(ampPos);
            double AvgPos = (Kpos.getMin() + Kpos.getMax())/2.0;
            datalogEXP.AvgPos.set(AvgPos);
            datalogEXP.PosError.set(Kpos.getSum());
            datalogEXP.minPitch.set(Kpitch.getMin());
            datalogEXP.maxPitch.set(Kpitch.getMax());
            double ampPitch = Kpitch.getMax() - Kpitch.getMin();
            datalogEXP.ampPitch.set(ampPitch);
            datalogEXP.PitchError.set(Kpitch.getSum());
            //datalogEXP.score.set(ampPitch*4.0+ampPos+Math.abs(AvgPos)); // low score wins!
            datalogEXP.score.set(8.0*Kpitch.getSum() + Kpos.getSum()); // low score wins!

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
            Kpos.resetSum();
            Kpitch.resetSum();
        }

        twb.loop(this);  // CALL MAIN TWB CONTROL SYSTEM

        telemetry.addLine(String.format("EXPERIMENT %d  OF TOTAL %d",count, NEXPERIMENTS));
        telemetry.addLine(" --- ");

        telemetry.addData("Kposition","%.7f",Kpos.getCurrent());
        telemetry.addData("Kvelo","%.7f", Kvelo.getCurrent());
        telemetry.addLine(" --- ");

        telemetry.addData("Kpitch","%.7f", Kpitch.getCurrent());
        telemetry.addData("KpitchRate","%.7f", KpitchRate.getCurrent());

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
                            KPIT,
                            KPOS,
                            KVELO,
                            KPITRATE,
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