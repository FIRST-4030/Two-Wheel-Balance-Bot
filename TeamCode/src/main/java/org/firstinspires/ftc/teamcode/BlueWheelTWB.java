package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Blue Wheeled Two Wheel Balancing Robot Class, with Arm.
 *  All of the robot unique constant are (should be) defined here
 */
public class BlueWheelTWB {
    private final TwoWheelBalanceController TWBController;

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
    private DatalogTWB datalogTWB; // datalog for full recording
    private boolean writeDatalog = false; // default is no log.  call method to write.
    public double MMPLoop = 8.0;
    public double DEGPLoop = 1.0;
    /**
     * TWB Constructor.  Call once.
      */
    public BlueWheelTWB(HardwareMap hardwareMap) {

        // The distance between the blue wheels is 300 mm
        // REVSPUR40PPR = 1120; // REV Core Hex Motor Pulses per Revolution at output shaft
        // COUNTS_PER_REV    = 2048.0 ;    // CUI ATM103 Encoder at most PPR
        // WHEELDIA = 203.0; // 8 inch wheel diameter (mm)
        // TICKSPERMM = (1120)/(203*Math.PI) = 1.75619; // REV SPUR 40:1, 8in wheels
        // Yaw PID terms: kp 0.45, ki 0.12, kd 0.05
        TWBController = new TwoWheelBalanceController(hardwareMap, 300.0, 203.0,
                1.75619, 0.45, 0.0, 0.05, 7, 3);

        VoltageSensor battery = hardwareMap.voltageSensor.get("Control Hub");
        // Get the current voltage, so that balance control is more consistent
        TWBController.setCurrentVoltage(battery.getVoltage());

        // These are the state terms for a two wheel balancing robot
        // Tune these using the DOE (Design of Experiments) opmode.
        // Both Kpos and Kvelo are negative when the center of mass is below the wheel axles
        // and positive when the CM is above (unstable). Sign does not change for Kpitch & KpitchRate
        //                            Kpos        Kvelo       Kpitch       KpitchRate
        TWBController.setBalanceTerms(0.020,0.018,-0.58,-0.028);
        //                                  0.016       0.015       -0.58           -0.025

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
        if (TWBController.getNewPitch() > 0.0) theArm.setArmAngle(ARMMAX);
        else theArm.setArmAngle(-125.0);  // -140 drives IMU pitch past zero, so using less arm angle

        theArm.updateArm(TWBController.getDeltaTime()); // This will make the arm move
    }

    /**
     * Start calls the TWB controller start
     */
    public void start() {
        TWBController.start();
        clawTimer.reset();
    }
    /**
     * TWB Main Loop method.  Call repeatedly while running. Contains balance control logic.
     * Teleoperated inputs are removed from this method, so it can be called in autonomous.
      */
    @SuppressLint("DefaultLocale")
    public void loop(OpMode theOpmode) {

        TWBController.setArmPitchTarget(pitchAngVec.getY(theArm.getAngle()));

        TWBController.loop(theOpmode);

        theArm.updateArm(TWBController.getDeltaTime()); // This will make the arm move

        if (writeDatalog) {
            datalogTWB.logPosPitch(TWBController.getPosition(), TWBController.getPosTarget(),
                    TWBController.getVelocity(), TWBController.getVeloTarget(),TWBController.getPitch(),
                    TWBController.getPitchTarget(), TWBController.getPitchRate(),
                    TWBController.getYaw(),TWBController.getYawTarget(),
                    TWBController.getPositionVolts(),TWBController.getPitchVolts(),
                    TWBController.getDeltaTime());
            datalogTWB.writeLineTWB();
        }
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

    /**
     * TWB method to provide user control of turning the robot.
     * @param deltaAngle a value from -1 to 1 in radians
     */
    public void turn_teleop(double deltaAngle) {
        // Robot Turning:
        // The right joystick turns the robot by adjusting the yaw PID turn setpoint
        TWBController.setYawTarget(TWBController.getYawTarget() - deltaAngle );
    }

    /**
     * TWB method translates the robot at the current angle by setting Position, Velocity & Pitch Targets.
     * @param forward value from -1 to 1 that is the forward or backward amount
     * @param degPerLoop robot pitch degrees per loop, multiplied by forward (6 is good)
     * @param mmPerLoop robot translation in mm per loop, multiplied by forward (7 is good)
     */
    public void translateDrive(double forward, double mmPerLoop, double degPerLoop) {

        // add some pitch to get it moving (degPerLoop of 6 results in gentle movement)
        TWBController.setAutoPitchTarget(forward * degPerLoop);

        // Update posTarget (mm) Note: this value * 50 = mm per second
        // mmPerLoop of 7 results in a gentle speed
        TWBController.setPosTarget( TWBController.getPosTarget() - forward * mmPerLoop );

        // Update the velocity target (mm/sec)
        //TWBController.setVeloTarget( -forward*(mmPerLoop/0.020));
        //TWBController.setVeloTarget( -forward*(mmPerLoop/TWBController.getDeltaTime()));
    }
    @SuppressLint("DefaultLocale")
    public void writeTelemetry(OpMode om) {
        om.telemetry.addLine(String.format("s Position Target %.1f ,Current %.1f (mm)",
                TWBController.getPosTarget(),TWBController.getPosition()));
        om.telemetry.addLine(String.format("s Velocity Target %.1f ,Current %.1f (mm/sec)",
                TWBController.getVeloTarget(),TWBController.getVelocity()));
        om.telemetry.addLine(String.format("Pitch Target %.1f ,Current %.1f (degrees)",
                TWBController.getPitchTarget(),TWBController.getPitch()));
        om.telemetry.addLine(String.format("Arm Angle Target %.1f ,Current %.1f (degrees)",
                theArm.getTargetAngle(),theArm.getAngle()));
        om.telemetry.addData("Current Voltage   ",TWBController.getCurrentVoltage());
    }

    public void closeClaw() {clawServo.setPosition(CLAWCLOSE);}
    public void openClaw() {clawServo.setPosition(CLAWOPEN);}
    public double getKpos() {return TWBController.getKpos();}
    public double getKpitch() {return TWBController.getKpitch();}
    public double getKvelo() {return TWBController.getKvelo();}
    public double getKpitchRate() {return TWBController.getKpitchRate();}
    public void setKpos(double k) {TWBController.setKpos(k);}
    public void setKpitch(double k) {TWBController.setKpitch(k);}
    public void setKpitchRate(double k) {TWBController.setKpitchRate(k);}
    public void setKvelo(double k) {TWBController.setKvelo(k);}
    public void setAutoPitchTarget(double target) {TWBController.setAutoPitchTarget(target);}
    public void setPosTarget(double pos) {TWBController.setPosTarget(pos);}
    public void setVeloTarget(double velo) {TWBController.setVeloTarget(velo);}
    public double getPos() {return TWBController.getPosition();}
    public double getPosTarget() {return TWBController.getPosTarget();}
    public double getVelocity() {return TWBController.getVelocity();}
    public double getPitch() {return TWBController.getPitch();}
    public double getPitchTarget() {return TWBController.getPitchTarget();}
    public double getPitchRate() {return TWBController.getPitchRate();}
    public double getPosVolts() {return TWBController.getPositionVolts();}
    public double getPitchVolts() {return TWBController.getPitchVolts();}
    public double getDeltaTime() {return TWBController.getDeltaTime();}
    public void writeDatalog(String LogName) {
        this.writeDatalog=true;
        datalogTWB = new DatalogTWB();
        datalogTWB.init(LogName);
    }
    public void setYawTarget(double yaw) {TWBController.setYawTarget(yaw);}
    public double getYawTarget() {return TWBController.getYawTarget();}
    public double getYaw() {return TWBController.getYaw();}
    public void imuReset() {TWBController.imuYawReset();}

}