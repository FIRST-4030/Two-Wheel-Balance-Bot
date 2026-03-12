package org.firstinspires.ftc.teamcode;

/**
 * DatalogTWB is a class for providing datalogging for the Two Wheeled Balancing Bot
 *  where the data is to be written every loop cycle.
 */
public class DatalogTWB {

    private DatalogTWBinside datalog; // create the data logger object

    /**
     * TWB init. Called once at initialization
     */
    public void init(String fileName) {
        datalog = new DatalogTWBinside(fileName);
    }
    public void logPosPitch(double pos, double posTarget, double pitch, double pitchTarget,
                            double posVolts, double pitchVolts, double dt) {
        // Data log
        // Note that the order in which we set datalog fields
        // does *not* matter! Order is configured inside the Datalog class constructor.
        datalog.pos.set(pos);
        datalog.posTarget.set(posTarget);
//        datalog.veloTarget.set(veloTarget);
        datalog.pitch.set(pitch);
        datalog.pitchTarget.set(pitchTarget);
//        datalog.pitchRATE.set(pitchRATE);
//        datalog.yaw.set(yaw);
//        datalog.yawTarget.set(yawTarget);
//        datalog.yawTheta.set(theta);
//        datalog.x.set(odometry.getX());
//        datalog.y.set(odometry.getY());
//        datalog.leftTicks.set(leftTicks / TICKSPERMM);
//        datalog.rightTicks.set(rightTicks); // TICKSPERMM
//        datalog.linVelo.set(odometry.getLinearVelocity());
//        datalog.avgLinVelo.set(linearVelocity); // this is a running average
        datalog.positionVolts.set(posVolts); // look for saturation when tuning
        datalog.pitchVolts.set(pitchVolts);  // look for saturation when tuning
//        datalog.totalVolts.set(totalPowerVolts);
//        datalog.yawPwr.set(yawPower * currentVoltage);
//        datalog.battery.set(battery.getVoltage());
//        datalog.armAngle.set(theArm.getAngle());
//        datalog.Kpos.set(Kpos);
//        datalog.Kvelo.set(Kvelo);
//        datalog.Kpitch.set(Kpitch);
//        datalog.KpitchRate.set(KpitchRate);
//        datalog.PosAmplitude.set(PosAmplitude);
        datalog.dt.set(dt);
    }
    public void writeLineTWB() {
        // The logged timestamp is taken when writeLine() is called.
        datalog.writeLine();
    }
    /**
     * Datalog class encapsulates all the fields that will go into the datalog.
     */
    public static class DatalogTWBinside {
        // The underlying datalogger object - it cares only about an array of loggable fields
        private final Datalogger datalogger;

        // These are all of the fields that we want in the datalog.
        // Note that order here is NOT important. The order is important in the setFields() call below
        public Datalogger.GenericField pitch = new Datalogger.GenericField("Pitch");
        public Datalogger.GenericField pitchTarget = new Datalogger.GenericField("PitchTarget");
        public Datalogger.GenericField pitchRATE = new Datalogger.GenericField("pitchRATE");
        public Datalogger.GenericField pos = new Datalogger.GenericField("PosCurrent");
        public Datalogger.GenericField posTarget = new Datalogger.GenericField("PosTarget");
        public Datalogger.GenericField veloTarget = new Datalogger.GenericField("VeloTarget");
        public Datalogger.GenericField yaw = new Datalogger.GenericField("Yaw");
        public Datalogger.GenericField yawTarget = new Datalogger.GenericField("yawTarget");
        public Datalogger.GenericField yawTheta = new Datalogger.GenericField("Theta");
        public Datalogger.GenericField x = new Datalogger.GenericField("x");
        public Datalogger.GenericField y = new Datalogger.GenericField("y");
        public Datalogger.GenericField leftTicks = new Datalogger.GenericField("leftMtrMM");
        public Datalogger.GenericField rightTicks = new Datalogger.GenericField("rightMtrMM");
        public Datalogger.GenericField rightODO = new Datalogger.GenericField("rightODOMM");
        public Datalogger.GenericField linVelo = new Datalogger.GenericField("linearVelo");
        public Datalogger.GenericField avgLinVelo = new Datalogger.GenericField("AvgLinearVelo");
        public Datalogger.GenericField positionVolts = new Datalogger.GenericField("positionVolts");
        public Datalogger.GenericField pitchVolts = new Datalogger.GenericField("pitchVolts");
        public Datalogger.GenericField totalVolts = new Datalogger.GenericField("totalVolts");
        public Datalogger.GenericField yawPwr = new Datalogger.GenericField("yawPower");
        public Datalogger.GenericField battery = new Datalogger.GenericField("Battery");
        public Datalogger.GenericField armAngle = new Datalogger.GenericField("armAngle");

        public Datalogger.GenericField PosAmplitude = new Datalogger.GenericField("PosAmplitude");
        public Datalogger.GenericField dt = new Datalogger.GenericField("DeltaTime");

        public DatalogTWBinside(String name) {
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
                            pitch,
                            pitchTarget,
                            //pitchRATE,
                            pos,
                            posTarget,
                            //veloTarget,
                            //yaw,
                            //yawTarget,
                            //yawTheta,
                            //x,
                            //y,
                            //leftTicks,
                            //rightTicks,
                            //rightODO,
                            //linVelo,
                            //avgLinVelo,
                            positionVolts,
                            pitchVolts,
                            //totalVolts,
                            //yawPwr,
                            //battery,
                            //armAngle,
                            //PosAmplitude,
                            dt
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
