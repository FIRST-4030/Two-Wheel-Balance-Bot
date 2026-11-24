package org.firstinspires.ftc.teamcode.Blue_Bot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ArmServo;
import org.firstinspires.ftc.teamcode.PiecewiseFunction;
import org.firstinspires.ftc.teamcode.TwoWheelBalanceController;

import java.util.Locale;

/**
 * Blue Wheeled Two Wheel Balancing Robot Class, with Arm.
 *  All of the robot unique constant are (should be) defined here
 */
public class BlueWheelTWB extends TwoWheelBalanceController{
    private final Servo clawServo;
    private boolean ClawIsClosed = false; //Claw boolean
    private final double CLAWCLOSE = 1.0; // servo value for closed claw
    private final double CLAWOPEN = 0.3;  // servo value for open claw
    private final ElapsedTime clawTimer = new ElapsedTime(); // Timer used with Claw


    //Handles the arm control, and adjusting the arm for the pitch of the robot
    private final ArmServo theArm;
    private final double ARMMIN = -140.0;
    private final double ARMMAX = 125.0;

    // PieceWise linear curve member for pitch angle vs arm angle
    final private PiecewiseFunction pitchAngVec = new PiecewiseFunction();


    /**
     * TWB Constructor.  Call once.
      */
    public BlueWheelTWB(HardwareMap hardwareMap) {
        super(hardwareMap, 300.0,
                1.75619, 0.45, 0.0, 0.05, 7, 1);
        // The distance between the blue wheels is 300 mm
        // REVSPUR40PPR = 1120; REV Core Hex Motor Pulses per Revolution at output shaft
        // WHEELDIA = 203.0;  8 inch wheel diameter (mm)
        // TICKSPERMM = (1120)/(203*Math.PI) = 1.75619;  REV SPUR 40:1, 8in wheels
        // Yaw PID terms: kp 0.45, ki 0.12, kd 0.05
        setMaxLinearVelocity(203.0, 40.0);

        this.imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));

        // These are the state terms for a two wheel balancing robot
        // Tune these using the DOE (Design of Experiments) opmode.
        // Both Kpos and Kvelo are negative when the center of mass is below the wheel axles
        // and positive when the CM is above (unstable). Sign does not change for Kpitch & KpitchRate
        //                    Kpos        Kvelo       Kpitch       KpitchRate

        //setBalanceTerms(0.001785,0.00125,-0.04845,-0.002125);
        setBalanceTerms(0.003,0.001,-0.051,-0.003);


        setTARGET_LOOP_MS(21.0); // need to test this for a good value

        setMaxAllowedVelocity(400.0);  // used for speed control
        setMaxAllowedAccel(5000.0);

        setDriveMotors(false, true, false);

        setZeroPitchTarget(-0.5); // zero angle, degrees, measure with PitchTune opmode

        setPitchLimitABS(90.0);  // Angle beyond which to shut down the opmode

        setVerticalCM(92.0);  // mm

        // Initialize the arm class
        // Determine servo values for two angle using the ServoTester opmode
        theArm = new ArmServo(hardwareMap, "arm_servo", 0.25, 90,
                0.68, -90, 40);
        theArm.setLimits(ARMMIN, ARMMAX); // physical limits to keep from breaking things

        /*
         * Arm Angle (degrees) vs. robot Pitch: (Pitch setpoint is a function of arm angle)
         * This keeps the Center of Gravity (CG) of Arm + Robot Body over the Robot Wheel axis.
         * The computation has been done externally (using OpenSCAD Mass Properties Simulator program)
         * and saved as a lookup Piecewise curve.
         */
        pitchAngVec.debug = false;
        pitchAngVec.setClampLimits(false);
        pitchAngVec.addElement(-160,6.26952); // new global arm angle is -153.73"
        pitchAngVec.addElement(-140,9.09704); // new global arm angle is -130.903"
        pitchAngVec.addElement(-120,10.459); // new global arm angle is -109.541"
        pitchAngVec.addElement(-100,10.4217); // new global arm angle is -89.5783"
        pitchAngVec.addElement(-80,9.22811); // new global arm angle is -70.7719"
        pitchAngVec.addElement(-60,7.16788); // new global arm angle is -52.8321"
        pitchAngVec.addElement(-40,4.51794); // new global arm angle is -35.4821"
        pitchAngVec.addElement(-20,1.52861); // new global arm angle is -18.4714"
        pitchAngVec.addElement(0,-1.57038); // new global arm angle is -1.57038"
        pitchAngVec.addElement(20,-4.55667); // new global arm angle is 15.4433"
        pitchAngVec.addElement(40,-7.20035); // new global arm angle is 32.7996"
        pitchAngVec.addElement(60,-9.25074); // new global arm angle is 50.7493"
        pitchAngVec.addElement(80,-10.4305); // new global arm angle is 69.5695"
        pitchAngVec.addElement(100,-10.4503); // new global arm angle is 89.5497"
        pitchAngVec.addElement(120,-9.06833); // new global arm angle is 110.932"
        pitchAngVec.addElement(140,-6.22194); // new global arm angle is 133.778"
        pitchAngVec.addElement(160,-2.205); // new global arm angle is 157.795"

        clawServo = hardwareMap.get(Servo.class, "clawServo");
    }

    /**
     * Set Arm Angle method.
      */
    public void setArmAngle(double armAngle) { theArm.setArmAngle(armAngle);  }

    /**
     *  TWB automatic self righting method.  Call repeatedly in initialization.
     */
    public void auto_right_loop() {
        // Check which way the robot is leaning and rotate the arm so that it will self-right
        if (getNewPitch() > 0.0) theArm.setArmAngle(ARMMAX);
        else theArm.setArmAngle(-125.0);  // -140 drives IMU pitch past zero, so using less arm angle

        theArm.updateArm(getDeltaTime()); // This will make the arm move
    }

    /**
     * Start calls the TWB controller start
     */
    public void start() {
        super.start();
        clawTimer.reset();
        imuYawPitchReset();
    }
    /**
     * TWB Main Loop method.  Call repeatedly while running. Contains balance control logic.
     * Teleoperated inputs are removed from this method, so it can be called in autonomous.
      */
    public void loopBlue(OpMode theOpmode) {

        setAddPitchTarget(pitchAngVec.getY(theArm.getAngle()));

        loop(theOpmode);

        theArm.updateArm(getDeltaTime()); // This will make the arm move

    }

    /**
     * TWB method that rotates the arm, if the scaler is not 0
     * @param velocityScalar velocity scalar from -1 to 1
     */
    public void arm_teleop(double velocityScalar) {
        if (Math.abs(velocityScalar) > 0.02 ) {
            //Increment target Arm angle
            double newAngle = theArm.getAngle() +  velocityScalar;
            theArm.setArmAngle(newAngle);
        }
    }

    /**
     * TWB method to provide user control of the claw.
     * @param toggle boolean, switch the claw state if true
     */
    public void claw_teleop(boolean toggle) {
        //Controls the claw boolean
        if (toggle) {
            if (ClawIsClosed)  openClaw(); // open
            else {
                closeClaw();  // close
                if (theArm.getAngle() < ARMMIN + 20.0) {
                    theArm.setArmAngle(theArm.getAngle() + 15.0);  // raise arm a bit to avoid runaway
                }
            }
            ClawIsClosed = !ClawIsClosed;
        }
    }

    /**
     * continuously opens and closes the claw when called
     */
    public void clawWave() {
        if (theArm.getAngle() > -100.0 && theArm.getAngle() < 100.0) {
            if (ClawIsClosed && clawTimer.seconds() > 0.3) {
                openClaw(); // open
                clawTimer.reset();
                ClawIsClosed = !ClawIsClosed;
            } else if (clawTimer.seconds() > 0.3) {
                closeClaw();
                clawTimer.reset();
                ClawIsClosed = !ClawIsClosed;
            }
        }
    }

    public void writeTelemetry(OpMode om) {
        om.telemetry.addLine(String.format(Locale.US,"s Position Target %.1f ,Current %.1f (mm)",
                getPosTarget(),getPos()));
        om.telemetry.addLine(String.format(Locale.US,"s Velocity Target %.1f ,Current %.1f (mm/sec)",
                getAcceleration(),getVelocity()));
        om.telemetry.addLine(String.format(Locale.US,"Pitch Target %.1f ,Current %.1f (degrees)",
                getPitchTarget(),getPitch()));
        om.telemetry.addLine(String.format(Locale.US,"Arm Angle Target %.1f ,Current %.1f (degrees)",
                theArm.getTargetAngle(),theArm.getAngle()));
    }
    public void closeClaw() {clawServo.setPosition(CLAWCLOSE);}
    public void openClaw() {clawServo.setPosition(CLAWOPEN);}

}