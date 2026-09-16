
package org.firstinspires.ftc.teamcode.C_Bot;

import static org.firstinspires.ftc.teamcode.C_Bot.C_TWB.GEARDOWNTIME;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private final double testDuration = 5.0; // seconds per experiment
    private final double DISTANCE = 250.0; // Travel distance for each test (mm)
    private boolean forward = true; // to keep track of direction
    private double virtualJoystick = 0.0;

    // Modify the Terms in init()
    private Term term1;  // generic names.  specific tests terms defined below
    private Term term2;
    private Term term3;

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

        twb.writeDatalog("C_DOE_bnf_Full"); // This log will be bigger

        // MODIFY THESE FOR THE EXPERIMENTS.
        term1 = new Term(0.010,0.014,3,twb.getKpos());  // Kpos

        //term2 = new Term(0.0024,0.0025,2,twb.getKvelo()); // Kvelo
        term2 = new Term(0.20,0.22,3,twb.getKpitch()); // Kpitch

        term3 = new Term(0.0044,0.0048,3,twb.getKpitchRate());

        NEXPERIMENTS = term1.getN() * term2.getN() * term3.getN();

        twb.init();
    }

    /**
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     * The Robot MOVES (balances) on init!!!
     */
    @Override
    public void init_loop() {
        telemetry.addLine("BACK AND FORTH DOE");
        telemetry.addLine(String.format(Locale.US, "  Will move %.1f mm",DISTANCE));
        telemetry.addLine(String.format(Locale.US, "TOTAL EXPERIMENTS %d",NEXPERIMENTS));
        telemetry.addLine(String.format(Locale.US, "TOTAL TIME %.2f sec",NEXPERIMENTS*testDuration));

        twb.init_loop();

        telemetry.update();
    }

    /**
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        twb.start();

        twb.setMaxAllowedVelocity(300.0);  // sets the max velocity

        resetRuntime();
        moveTimer.reset();
    }

    /**
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    @SuppressLint("DefaultLocale")
    public void loop() {
        twb.startCycleTImer();

        if (moveTimer.seconds() < 0.03) {
            // set the new DOE K terms
            twb.setKpos(-term1.getCurrent());
            //twb.setKvelo(-term2.getCurrent());
            twb.setKpitch(-term2.getCurrent());
            twb.setKpitchRate(-term3.getCurrent());

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
            term1.updateMinMax(thisPos);

            // build the minimum amplitude "box" on the pitch wave
            term2.updateMinMax(thisPitch);

            // Integrate the position and pitch errors over time
            term1.updateSum(thisPos, twb.getPosTarget(), thisDT);
            term2.updateSum(thisPitch, twb.getPitchTarget(), thisDT);

            // turn off the virtual joystick when distance is reached
            if (forward && twb.getPosTarget() >= DISTANCE) virtualJoystick = 0.0;
            if (!forward && twb.getPosTarget() <= 0.0) virtualJoystick = 0.0;

        } else if(moveTimer.seconds() > testDuration ) {
            virtualJoystick = 0.0;
            // At the end of the experiment, only once, log data and do resets

            // datalog - one line for each experiment
            datalogEXP.count.set(count);
            datalogEXP.term1.set(term1.getCurrent());
            datalogEXP.term2.set(term2.getCurrent());
            datalogEXP.term3.set(term3.getCurrent());

            datalogEXP.PosError.set(term1.getSum());

            datalogEXP.PitchError.set(term2.getSum()*10.0); // amplify to better match PosError

            // The logged timestamp is taken when writeLine() is called.
            datalogEXP.writeLine();

            // set up for the next experiment
            term1.next();
            if(count % term1.getN() == 0) {
                term2.next();
            }
            if((count % (term1.getN()*term2.getN())) == 0) {
                term3.next();
            }

            moveTimer.reset();
            forward = !forward;

            count += 1;

            term1.resetMinMax();
            term2.resetMinMax();
            term1.resetSum();
            term2.resetSum();
        }
        // correct robot Y position by updating the yaw target
        double yawCorrection = Math.atan2(500.0,twb.getY());
        if (forward) twb.setYawTarget(-yawCorrection);
        else twb.setYawTarget(yawCorrection);

        // Translate the robot
        twb.translateDrive(virtualJoystick);

        telemetry.addLine(String.format("EXPERIMENT %d  OF TOTAL %d",count, NEXPERIMENTS));
        telemetry.addLine(" --- ");

        telemetry.addData("term1 (Kpos)","%.5f", term1.getCurrent());
        telemetry.addLine(" --- ");

        telemetry.addData("term2 (Kpitch)","%.5f", term2.getCurrent());
        //telemetry.addData("KpitchRate","%.7f", UNUSED2.getCurrent());
        telemetry.addLine(" --- ");
        telemetry.addData("term3 (KpitchRate)","%.5f", term3.getCurrent());

        telemetry.update();

        if (count > NEXPERIMENTS) {
            twb.moveGearDown();
            if(moveTimer.seconds() > GEARDOWNTIME) requestOpModeStop(); // Stop the opmode
        }
        twb.loopC(this);  // CALL MAIN TWB CONTROL SYSTEM

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
        public Datalogger.GenericField term1 = new Datalogger.GenericField("Kpos");
        public Datalogger.GenericField term2 = new Datalogger.GenericField("Kpitch");
        public Datalogger.GenericField term3 = new Datalogger.GenericField("KPrate");

        //public Datalogger.GenericField minPos = new Datalogger.GenericField("minPos");
        //public Datalogger.GenericField maxPos = new Datalogger.GenericField("maxPos");
        //public Datalogger.GenericField ampPos = new Datalogger.GenericField("ampPos");
        //public Datalogger.GenericField AvgPos = new Datalogger.GenericField("AVG_Pos");
        public Datalogger.GenericField PosError = new Datalogger.GenericField("Pos_Error");

        //public Datalogger.GenericField minPitch = new Datalogger.GenericField("minPitch");
        //public Datalogger.GenericField maxPitch = new Datalogger.GenericField("maxPitch");
        //public Datalogger.GenericField ampPitch = new Datalogger.GenericField("ampPitch");
        public Datalogger.GenericField PitchError = new Datalogger.GenericField("Pitch_Error");

        //public Datalogger.GenericField score = new Datalogger.GenericField("SCORE");


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
                            term1,
                            term2,
                            term3,
                            //minPos,
                            //maxPos,
                            //ampPos,
                            //AvgPos,
                            PosError,
                            //minPitch,
                            //maxPitch,
                            //ampPitch,
                            PitchError
                            //score
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