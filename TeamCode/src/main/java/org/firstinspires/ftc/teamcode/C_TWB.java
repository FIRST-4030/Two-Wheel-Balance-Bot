package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Black Wheeled Two Wheel Balancing Robot Class
 *  All of the robot unique constant are (should be) defined here
 */
public class C_TWB {
    private final TwoWheelBalanceController TWBController;

    private final Servo leftGearServo;
    private final Servo rightGearServo;

    private final static double RIGHTDOWN = 0.88; // servo value
    private final static double RIGHTUP = 0.40;  // servo value
    private final static double LEFTDOWN = 0.82; // servo value
    private final static double LEFTUP = 0.30;  // servo value
    private final ElapsedTime clawTimer = new ElapsedTime(); // Timer used with Claw

    private DatalogTWB CdatalogTWB; // datalog for full recording
    private boolean writeDatalog = false; // default is no log.  call method to write.
    public double MMPLoop = 8.0;
    public double DEGPLoop = 1.0;
    /**
     * TWB Constructor.  Call once.
      */
    public C_TWB(HardwareMap hardwareMap) {
        // COUNTS_PER_REV    = 2048.0 ;    // CUI ATM103 Encoder at most PPR. Getting 4 times this.
        // WHEELDIA = 96.0; // goBilda Rhino wheels
        // TICKSPERMM = (8192)/(96*Math.PI) = 27.16244;
        // Yaw PID terms: kp 0.45, ki 0.12, kd 0.05
        TWBController = new TwoWheelBalanceController(hardwareMap, 246.0, 96.0,
                27.16244, 0.45, 0.0, 0.05, 1, 1);

        // These are the state terms for a two wheel balancing robot
        // Tune these using the DOE (Design of Experiments) opmode.
        // Both Kpos and Kvelo are negative when the center of mass is below the wheel axles
        // and positive when the CM is above (unstable). Sign does not change for Kpitch & KpitchRate
        //                            Kpos        Kvelo       Kpitch       KpitchRate
        TWBController.setBalanceTerms(0.030,0.0001,-0.1,-0.001);

        leftGearServo = hardwareMap.get(Servo.class, "leftGearServo");
        rightGearServo = hardwareMap.get(Servo.class, "rightGearServo");
    }

    public void init() {
        // put the gear down
        leftGearServo.setPosition(LEFTDOWN);
        rightGearServo.setPosition(RIGHTDOWN);
    }
    /**
     * Start is called once after play is pushed and calls the TWB controller start
     */
    public void start() {
        TWBController.start();
        leftGearServo.setPosition(LEFTUP);
        rightGearServo.setPosition(RIGHTUP);
    }
    /**
     * TWB Main Loop method.  Call repeatedly while running. Contains balance control logic.
     * Teleoperated inputs are removed from this method, so it can be called in autonomous.
      */
    @SuppressLint("DefaultLocale")
    public void loop(OpMode theOpmode) {

        TWBController.loop(theOpmode);

        if (writeDatalog) {
            CdatalogTWB.logPosPitch(TWBController.getPosition(), TWBController.getPosTarget(),
                    TWBController.getVelocity(), TWBController.getVeloTarget(),TWBController.getPitch(),
                    TWBController.getPitchTarget(), TWBController.getPitchRate(),
                    TWBController.getYaw(),TWBController.getYawTarget(),
                    TWBController.getPositionVolts(),TWBController.getPitchVolts(),
                    TWBController.getDeltaTime());
            CdatalogTWB.writeLineTWB();
        }
    }

    public void stop(OpMode om) {
        // put the gear down and wait for a bit
        leftGearServo.setPosition(LEFTDOWN);
        rightGearServo.setPosition(RIGHTDOWN);
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < 0.5) {
            om.telemetry.addLine("stopping");
            om.telemetry.update();
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
        om.telemetry.addData("Current Voltage   ",TWBController.getCurrentVoltage());
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
        CdatalogTWB = new DatalogTWB();
        CdatalogTWB.init(LogName);
    }
    public void setYawTarget(double yaw) {TWBController.setYawTarget(yaw);}
    public double getYawTarget() {return TWBController.getYawTarget();}
    public double getYaw() {return TWBController.getYaw();}
    public void imuReset() {TWBController.imuYawReset();}
    public int getLeftTicks() {return TWBController.getLeftTicks();}
    public int getRightTicks() {return TWBController.getRightTicks();}

    public void setDriveMotors(boolean leftForward, boolean rightForward,boolean revEncoders) {
        TWBController.setDriveMotors(leftForward,rightForward,revEncoders);
    }
}