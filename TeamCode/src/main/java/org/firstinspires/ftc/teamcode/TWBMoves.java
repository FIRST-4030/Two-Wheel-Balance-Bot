package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Two Wheel Bot Moves object.
 *  Methods to drive specified distance in specified time using curves to smooth the motion.
 *  The position curve provide velocity ramping. The pitch curve is currently manually derived.
 */
public class TWBMoves {
    // members
    public boolean reverseDir = false; // for running backwards
    
    // PieceWise member for position profiling
    private PiecewiseFunction posVector = new PiecewiseFunction();
    private PiecewiseFunction veloVector = new PiecewiseFunction();
    
    // PieceWise member for pitch profiling
    final private PiecewiseFunction pitchVector = new PiecewiseFunction();

    private double priorTime=0.0;
    private double priorPosition=0.0;

    /**
     * Constructor.  Initializes the position and pitch curves.
      */
    public TWBMoves(double time, double amplitude) {

        // Position profile to travel 1 cm in 1 sec, 
        //   using an approximate trapezoidal velocity profile
        posVector.debug = false;
//        posVector.addElement(.00,.00);
//        posVector.addElement(.05,.01);
//        posVector.addElement(.10,.02);
//        posVector.addElement(.15,.04);
//        posVector.addElement(.20,.08);
//        posVector.addElement(.25,.14);
//        posVector.addElement(.30,.22);
//        posVector.addElement(.35,.31);
//        posVector.addElement(.40,.40);
//        posVector.addElement(.45,.50);
//        posVector.addElement(.50,.60);
//        posVector.addElement(.55,.69);
//        posVector.addElement(.60,.78);
//        posVector.addElement(.65,.86);
//        posVector.addElement(.70,.91);
//        posVector.addElement(.75,.96);
//        posVector.addElement(.80,.98);
//        posVector.addElement(.85,.99);
//        posVector.addElement(.90,1.00);
//        posVector.addElement(.95,1.00);
//        posVector.addElement(1.00,1.00);
        //fillSineWave(20,time,amplitude,posVector);
        int pieces=20;
        for (int i=0; i <= pieces; i++)  {
            double iTime =  ((double)i /(double)pieces) * time;
            double angle =  ((double)i /(double)pieces) * Math.PI - Math.PI/2.0;
            double Dist = (amplitude/2) * (Math.sin((angle))+1.0 );
            posVector.addElement( iTime,    Dist);
        }
        //fillDerivative(posVector,veloVector);
        for (int i=0; i < pieces; i++) {
            double slope = (posVector.getElementY(i)-posVector.getElementY(i+1))/
                    (posVector.getElementX(i)-posVector.getElementX(i+1));
            veloVector.addElement(posVector.getElementX(i),slope);
        }

        veloVector.addElement(posVector.getElementX(pieces),0);
        //veloVector.debug = false;

        // Pitch profile to travel 1 cm in 1 sec, 
        //   using an ... manually derived curve.  Trial and error...
        pitchVector.debug = false;
        pitchVector.addElement(.00,.00);
        pitchVector.addElement(.2*time,-1.5); // DOE
        //pitchVector.addElement(.02,0.0);
        pitchVector.addElement(.5*time,-0.1);  // DOE
        //pitchVector.addElement(.95,0.0);
        pitchVector.addElement(.8*time,1.5);   // DOE
        pitchVector.addElement(time,.00);
    }

    /**
     * Set the Pitch Profile for the back-n-forth DOE
     * @param newYs new values for the pitch piecewise curve, indicated above
     */
    public void setPitchVector(double[] newYs) {
        pitchVector.setElement(1, 0.01,newYs[0]);
        pitchVector.setElement(2, 0.5,newYs[1]);
        pitchVector.setElement(3, 0.975,newYs[2]);
    }

    /**
     * lineMove method, to be called continuously for the duration of the requested move.
     * Scales the position and pitch curves to the requested values.
     * @param currentTime (seconds) from start of move
     * @param startingS (mm) starting position S
     * @return array [3] containing posTarget, pitchTarget, and velocityTarget
     */
    public double[] lineMove(double currentTime, double startingS) {
        double posTarget;
        double pitchTarget;
        double velocity;
        int sign; // for direction

        if (reverseDir) sign = -1;
        else sign = 1;

        posTarget = sign*posVector.getY(currentTime) + startingS;

        velocity = sign*veloVector.getY(currentTime);

        pitchTarget = sign*pitchVector.getY(currentTime);

        return new double[] {posTarget, pitchTarget, velocity};
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
    public double[] OLDlineMove(double distance, double totalTime, double currentTime, double startingS) {
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

        velocity = (posTarget-priorPosition)/(currentTime-priorTime);
        priorPosition=posTarget;
        priorTime=currentTime;
        //velocity = 0.0;

        max_v = 2.0 * distance / totalTime;
        max_a = 4.0 * max_v / totalTime;
        pitchScaler = ( max_a / 8.0) / 10.0; // convert from cm to mm
        pitchTarget = sign*pitchScaler*pitchVector.getY(currentTime/totalTime);
        if (pitchTarget > 20.0) pitchTarget = 20.0;
        else if (pitchTarget < -20.0) pitchTarget = -20.0;

        return new double[] {posTarget,pitchTarget, velocity};
    }

    /**
     * Builds a sine curve from -PI/2 to PI/2 (180 degrees)
     * @param pieces number of pieces in the curve, from 2 to 100
     * @param time The x axis value at PI/2.  Starts at zero.
     * @param amplitude The y axis value at PI/2.  Starts at zero
     * @param curve The sine curve as a piecewiseFunction
     */
    public void fillSineWave(int pieces, double time, double amplitude,
                             PiecewiseFunction curve ) {

        if (pieces < 3) pieces = 2;
        if (pieces > 100) pieces = 100;
        if (time < 0) time -= time;
        if (time == 0) time = 1.0;

        for (int i=0; i <= pieces; i++)  {
            double iTime =  ((double)i /(double)pieces) * time;
            double angle =  ((double)i /(double)pieces) * Math.PI - Math.PI/2.0;
            double Dist = (amplitude/2) * (Math.sin((angle))+1.0 );
            curve.addElement( iTime,    Dist);
        }
    }

    /**
     * Builds a derivative PiecewiseFunction from a given PiecewiseFunction
     * @param curve PiecewiseFunction
     * @param derivative PiecewiseFunction
     */
    public void fillDerivative(PiecewiseFunction curve, PiecewiseFunction derivative) {
        int pieces = curve.getSize();

        for (int i=0; i < pieces; i++) {
            double slope = (curve.getElementY(i)-curve.getElementY(i+1))/
                    (curve.getElementX(i)-curve.getElementX(i+1));
            derivative.addElement(curve.getElementX(i),slope);
        }

        derivative.addElement(curve.getElementX(pieces),0.0);
    }

    @SuppressLint("DefaultLocale")
    public void writeTelemetry(OpMode om) {
        for(int i=0;i<posVector.getSize();i++) {
            om.telemetry.addLine(String.format("Move Vector i %d ,t %.1f ,pos %.1f ,velo %.1f",
                    i, posVector.getElementX(i),posVector.getElementY(i),veloVector.getElementY(i)));
        }
    }

}
