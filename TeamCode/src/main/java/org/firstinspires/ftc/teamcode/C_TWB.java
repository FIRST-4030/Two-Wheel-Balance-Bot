package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

/**
 * Black Wheeled Two Wheel Balancing Robot Class
 *  All of the robot unique constant are (should be) defined here
 */
public class C_TWB {
    private final TwoWheelBalanceController TWBController;

    private boolean GearDown = true;
    private final Servo leftGearServo;
    private final Servo rightGearServo;

    private final static double RIGHTDOWN = 0.10; // servo value
    private final static double RIGHTUP = 0.68;  // servo value
    private final static double LEFTDOWN = 0.92; // servo value.  DETACHED
    private final static double LEFTUP = 0.40;  // servo value   DETACHED
    private final ElapsedTime gearTimer = new ElapsedTime(); // Timer used with Claw

    private DatalogTWB CdatalogTWB; // datalog for full recording
    private boolean writeDatalog = false; // default is no log.  call method to write.
    public double MMPLoop = 5.0; // 8 is large for pinpoint
    public double DEGPLoop = 2.0;

    private final DcMotor flywheel;
    private final ElapsedTime shotTimer = new ElapsedTime(); // Timer used for shooting
    private boolean shooting = false;
    private boolean collecting = false;
    /**
     * TWB Constructor.  Call once.
      */
    public C_TWB(HardwareMap hardwareMap) {
        // COUNTS_PER_REV    = 2048.0 ;    // CUI ATM103 Encoder at most PPR. Getting 4 times this.
        // WHEELDIA = 96.0; // goBilda Rhino wheels
        // TICKSPERMM = (8192)/(96*Math.PI) = 27.16244;
        // Yaw PID terms: kp 0.45, ki 0.12, kd 0.05
        TWBController = new TwoWheelBalanceController(hardwareMap, 246.0,
                27.16244, 0.45, 0.0, 0.05, 6, 1,
                TwoWheelBalanceController.Robot.CPin);

        // These are the state terms for a two wheel balancing robot
        // Tune these using the DOE (Design of Experiments) opmode.
        // Both Kpos and Kvelo are negative when the center of mass is below the wheel axles
        // and positive when the CM is above (unstable). Sign does not change for Kpitch & KpitchRate
        //                            Kpos        Kvelo       Kpitch       KpitchRate
        TWBController.setBalanceTerms(-0.01,-0.0022,-0.21,-0.0044);
        //                                  0.0044       0.0024     -0.085   -0.0066 -0.007 <= MAX pr

        TWBController.setArmPitchTarget(-0.5); // measure with C_Pitch_Fuzz opmode

        //TWBController.setDriveMotors(true,false,true); // REV IMU
        TWBController.setDriveMotors(false,true,false); // Pinpoint

        leftGearServo = hardwareMap.get(Servo.class, "leftGearServo");
        rightGearServo = hardwareMap.get(Servo.class, "rightGearServo");

        flywheel = hardwareMap.get(DcMotor.class, "fly");
        flywheel.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void init() {
        moveGearDown();
    }

    public void init_loop() {
        TWBController.updatePinpoint();
    }
    /**
     * Start is called once after play is pushed and calls the TWB controller start
     */
    public void start() {
        TWBController.start();
        moveGearUp();
    }
    /**
     * TWB Main Loop method.  Call repeatedly while running. Contains balance control logic.
     * Teleoperated inputs are removed from this method, so it can be called in autonomous.
      */
    public void loop(OpMode theOpmode) {

        if (!GearDown) {
            TWBController.loop(theOpmode); // balancing
        } else { // gear is down or going down
            if (gearTimer.seconds() < 0.4) {
                TWBController.loop(theOpmode); // keep balancing while going down
            } else {
                TWBController.setMotorsZero();
            }
        }

        shoot_loop(); // check if we are shooting

        if (writeDatalog) {
            CdatalogTWB.logPosPitch(TWBController.getPosition() ,TWBController.getPosTarget(),
                    TWBController.getVelocity(), TWBController.getVeloTarget(),TWBController.getPitch(),
                    TWBController.getPitchTarget(), TWBController.getPitchRate(),
                    TWBController.getYaw(),TWBController.getYawTarget(),
                    TWBController.getPositionVolts(),TWBController.getPitchVolts(),
                    TWBController.getDeltaTime());
            CdatalogTWB.writeLineTWB();
        }
    }

    public void moveGearUp() {
        leftGearServo.setPosition(LEFTUP);
        rightGearServo.setPosition(RIGHTUP);
        GearDown = false;
    }
    public void moveGearDown() {
        // put the gear down and wait for a bit
        leftGearServo.setPosition(LEFTDOWN);
        rightGearServo.setPosition(RIGHTDOWN);
        GearDown = true;
        gearTimer.reset(); // start the timer
    }
    public boolean isGearDown() {return GearDown;}

    public void setFlywheel(double power) {
        flywheel.setPower(power);
    }

    public void shootFlywheel() {
        shooting = true;
        shotTimer.reset();
    }

    public void collectFlywheel() {
        collecting = true;
        flywheel.setPower(0.6);
    }

    public void flywheelOff() {
        collecting = false;
        shooting = false;
    }
    private void shoot_loop() {

        if (shooting && shotTimer.seconds() < 1.0) {
            flywheel.setPower(-0.6);
        } else {
            shooting = false;
        }
        if (!shooting && !collecting) flywheel.setPower(0.0);
    }

    /**
     * TWB method to provide user control of turning the robot.
     * @param deltaAngle a value from -1 to 1 in radians
     */
    public void turn_teleop(double deltaAngle) {
        // Robot Turning:
        // The right joystick turns the robot by adjusting the yaw PID turn setpoint
        TWBController.setYawTarget(TWBController.getYawTarget() + deltaAngle );
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
    public void writeTelemetry(OpMode om) {
        om.telemetry.addLine(String.format(Locale.US, "s Position Target %.1f ,Current %.1f (mm)",
                TWBController.getPosTarget(),TWBController.getPosition()));
        om.telemetry.addLine(String.format(Locale.US, "s Velocity Target %.1f ,Current %.1f (mm/sec)",
                TWBController.getVeloTarget(),TWBController.getVelocity()));
        om.telemetry.addLine(String.format(Locale.US, "Pitch Target %.1f ,Current %.1f (DEGREES)",
                TWBController.getPitchTarget(),TWBController.getPitch()));
        om.telemetry.addLine(String.format(Locale.US, "Yaw Target %.1f ,Current %.1f (RADIANS)",
                TWBController.getYawTarget(),TWBController.getYaw()));
        om.telemetry.addData("Left Ticks   ",TWBController.getLeftTicks());
        om.telemetry.addData("Right Ticks   ",TWBController.getRightTicks());
    }

    public double getKpos() {return TWBController.getKpos();}
    public double getKpitch() {return TWBController.getKpitch();}
    public double getKvelo() {return TWBController.getKvelo();}
    public double getKpitchRate() {return TWBController.getKpitchRate();}
    public void setKpos(double k) {TWBController.setKpos(k);}
    public void setKpitch(double k) {TWBController.setKpitch(k);}
    public void setKpitchRate(double k) {TWBController.setKpitchRate(k);}
    public void setKvelo(double k) {TWBController.setKvelo(k);}
    public void setAutoPitchTarget(double target) {TWBController.setAutoPitchTarget(target);}
    public double getPos() {return TWBController.getPosition();}
    public double getVelocity() {return TWBController.getVelocity();}
    public double getPitch() {return TWBController.getPitch();}
    public double getPitchTarget() {return TWBController.getPitchTarget();}
    public double getDeltaTime() {return TWBController.getDeltaTime();}
    public void writeDatalog(String LogName) {
        this.writeDatalog=true;
        CdatalogTWB = new DatalogTWB();
        CdatalogTWB.init(LogName);
    }
    public double getYaw() {return TWBController.getYaw();}
    public void setPosTarget(double pos) {TWBController.setPosTarget(pos);}

}