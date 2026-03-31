package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Two Wheel Bot Move object.
 *  Methods to drive specified distance in specified time using curves to smooth the motion.
 *  Position, velocity and pitch curves are used.
 */
public class TWBMove {
    // members
    public boolean reverseDir = false; // for running backwards
    
    private final PiecewiseFunction posCurve = new PiecewiseFunction();
    private final PiecewiseFunction veloCurve = new PiecewiseFunction();
    private final PiecewiseFunction pitchCurve = new PiecewiseFunction();

    /**
     * Constructor.  Initializes the position and pitch curves.
     * @param time seconds
     * @param distance how far to move in mm
      */
    public TWBMove(double time, double distance) {

        posCurve.debug = false;
        fillSineWave(20,time,distance,(- Math.PI/2.0), Math.PI, true, posCurve);

        fillDerivative(posCurve,veloCurve);
        veloCurve.debug = false;

        // Pitch profile to travel 1 cm in 1 sec, 
        //   using an ... manually derived curve.  Trial and error...
        pitchCurve.debug = false;
        fillSineWave(10,time/2.0,4.0,(-Math.PI), Math.PI, false,pitchCurve);
        //pitchCurve.addElement(0.0,.00);
        pitchCurve.addElement(time,.00);
//        pitchVector.addElement(.00,.00);
//        pitchVector.addElement(.01*time,-2.0); // DOE
//        //pitchVector.addElement(.02,0.0);
//        pitchVector.addElement(.25*time,-3.0);  // DOE
//        //pitchVector.addElement(.95,0.0);
//        pitchVector.addElement(.4*time,-2.0);   // DOE
//        pitchVector.addElement(.5*time,0.0);   // DOE
//        pitchVector.addElement(time,.00);
    }

    /**
     * Specific method that modifies the Pitch Profile for the back-n-forth DOE
     * @param newYs new values for the pitch piecewise curve, indicated above
     */
    public void setPitchCurve(double[] newYs) {
        pitchCurve.setElement(1, pitchCurve.getElementX(1), newYs[0]);
        pitchCurve.setElement(2, pitchCurve.getElementX(2), newYs[1]);
        pitchCurve.setElement(3, pitchCurve.getElementX(3), newYs[2]);
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

        posTarget = sign* posCurve.getY(currentTime) + startingS;

        velocity = sign* veloCurve.getY(currentTime);

        pitchTarget = sign* pitchCurve.getY(currentTime);

        return new double[] {posTarget, pitchTarget, velocity};
    }

    /**
     * Builds a sine curve from -PI/2 to PI/2 (180 degrees)
     * @param pieces number of pieces in the curve, from 2 to 100
     * @param time how long the move should take in seconds.  Starts at zero.
     * @param amplitude The y axis value at PI/2.  Starts at zero
     * @param period the periodic length of the curve, in radians
     * @param phase phase shift of the sine function, in radians
     * @param curve The sine curve as a piecewiseFunction
     */
    public void fillSineWave(int pieces, double time, double amplitude, double phase,
                             double period, boolean shift,PiecewiseFunction curve ) {

        double Dist;

        if (pieces < 3) pieces = 2;
        if (pieces > 100) pieces = 100;
        if (time < 0) time -= time;
        if (time == 0) time = 1.0;

        for (int i=0; i <= pieces; i++)  {
            double iTime =  ((double)i /(double)pieces) * time;
            double angle =  ((double)i /(double)pieces) * period + phase;
            if (shift) Dist = (amplitude/2) * (Math.sin((angle))+1.0 );
            else Dist = amplitude * Math.sin(angle);
            curve.addElement( iTime,    Dist);
        }
    }

    /**
     * Builds a derivative PiecewiseFunction from a given curve, forces both ends to zero slope.
     * @param curve PiecewiseFunction
     * @param derivative PiecewiseFunction
     */
    public void fillDerivative(PiecewiseFunction curve, PiecewiseFunction derivative) {
        int pieces = curve.getSize()-1; // getSize returns points, we need pieces

        derivative.addElement(0.0,0.0);
        double slope1 = (curve.getElementY(0)- curve.getElementY(1))/
                (curve.getElementX(0)- curve.getElementX(1));

        for (int i=1; i < pieces; i++) {
            double slope2 = (curve.getElementY(i)- curve.getElementY(i+1))/
                    (curve.getElementX(i)- curve.getElementX(i+1));
            derivative.addElement(curve.getElementX(i),(slope1+slope2)/2.0);
            slope1=slope2;
        }
        derivative.addElement(curve.getElementX(pieces),0.0);
    }

    public void newPitchCurveY(double scale) {
        int points = pitchCurve.getSize();

        for(int i=1; i<points; i++) {
            double newY = pitchCurve.getElementY(i) * scale;
            pitchCurve.setElement(i,pitchCurve.getElementX(i), newY);
        }
    }
    @SuppressLint("DefaultLocale")
    public void writeTelemetry(OpMode om) {
        for(int i = 0; i< posCurve.getSize(); i++) {
//            om.telemetry.addLine(String.format("Move i %d ,t %.1f ,pos %.1f ,velo %.1f,pitch %.1f",
//                    i, posCurve.getElementX(i), posCurve.getElementY(i), veloCurve.getElementY(i),
//                    pitchCurve.getElementY(i)));
            om.telemetry.addLine(String.format("Move i %d ,t %.1f ,pos %.1f ,velo %.1f",
                    i, posCurve.getElementX(i), posCurve.getElementY(i), veloCurve.getElementY(i)));
        }
    }

}
