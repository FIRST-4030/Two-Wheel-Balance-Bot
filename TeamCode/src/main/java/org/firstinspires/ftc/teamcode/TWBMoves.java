package org.firstinspires.ftc.teamcode;

/**
 * Two Wheel Bot Moves object.
 *  Methods to drive specified distance in specified time using curves to smooth the motion.
 *  The position curve provide velocity ramping. The pitch curve is currently manually derived.
 */
public class TWBMoves {
    // members
    public boolean reverseDir = false; // for running backwards
    
    // PieceWise member for position profiling
    final private PiecewiseFunction posVector = new PiecewiseFunction();
    
    // PieceWise member for pitch profiling
    final private PiecewiseFunction pitchVector = new PiecewiseFunction();

    private double priorTime=0.0;
    private double priorPosition=0.0;

    /**
     * Constructor.  Initializes the position and pitch curves.
      */
    public TWBMoves() {

        // Position profile to travel 1 cm in 1 sec, 
        //   using an approximate trapezoidal velocity profile
        posVector.debug = false;
        posVector.addElement(.00,.00);
        posVector.addElement(.05,.01);
        posVector.addElement(.10,.02);
        posVector.addElement(.15,.04);
        posVector.addElement(.20,.08);
        posVector.addElement(.25,.14);
        posVector.addElement(.30,.22);
        posVector.addElement(.35,.31);
        posVector.addElement(.40,.40);
        posVector.addElement(.45,.50);
        posVector.addElement(.50,.60);
        posVector.addElement(.55,.69);
        posVector.addElement(.60,.78);
        posVector.addElement(.65,.86);
        posVector.addElement(.70,.91);
        posVector.addElement(.75,.96);
        posVector.addElement(.80,.98);
        posVector.addElement(.85,.99);
        posVector.addElement(.90,1.00);
        posVector.addElement(.95,1.00);
        posVector.addElement(1.00,1.00);

        // Pitch profile to travel 1 cm in 1 sec, 
        //   using an ... manually derived curve.  Trial and error...
        pitchVector.debug = false;
        pitchVector.addElement(.00,.00);
        pitchVector.addElement(.05,-1.20); // determined by test
        pitchVector.addElement(.75,0.0);
        pitchVector.addElement(1.00,.00);
    }

    /**
     * lineMove method, to be called continuously for the duration of the requested move.
     * Scales the position and pitch curves to the requested values.
     * @param distance (mm) requested distance
     * @param totalTime (seconds) requested time
     * @param currentTime (seconds) from start of move
     * @param startingS (mm) starting position S
     * @return array [3] containing posTarget, pitchTarget, and velocityTarget
     */
    public double[] lineMove(double distance, double totalTime, double currentTime, double startingS) {
        double max_v;
        double max_a;
        double posTarget;
        double pitchTarget;
        double pitchScaler;
        double velocity;
        int sign; // for direction

        if (reverseDir) sign = -1;
        else sign = 1;

        posTarget = sign*distance*posVector.getY(currentTime/totalTime) + startingS;

        //velocity = (posTarget-priorPosition)/(currentTime-priorTime);
        priorPosition=posTarget;
        priorTime=currentTime;
        velocity = 0.0;

        max_v = 2.0 * distance / totalTime;
        max_a = 4.0 * max_v / totalTime;
        pitchScaler = ( max_a / 8.0) / 10.0; // convert from cm to mm
        pitchTarget = sign*pitchScaler*pitchVector.getY(currentTime/totalTime);
        if (pitchTarget > 20.0) pitchTarget = 20.0;
        else if (pitchTarget < -20.0) pitchTarget = -20.0;

        return new double[] {posTarget,pitchTarget, velocity};
    }

}
