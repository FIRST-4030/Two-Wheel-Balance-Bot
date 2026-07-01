package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

/**
 * Black Wheeled Two Wheel Balancing Robot Class
 *  Extends TWB class
 * Robot Details: goBilda 9.6 cm wheels, goBilda 26.9:1 motors, 1:1 belt drive
 */
public class C_TWB extends TwoWheelBalanceController{
    //private final TwoWheelBalanceController TWBController;

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
    public double DEGPLoop = -0.5;

    private final DcMotor flywheel;
    private final ElapsedTime shotTimer = new ElapsedTime(); // Timer used for shooting
    private boolean shooting = false;
    private boolean collecting = false;
    /**
     * TWB Constructor.  Call once.
      */
    public C_TWB(HardwareMap hardwareMap) {
        super(hardwareMap, 246.0,
                27.16244, 0.45, 0.0, 0.05, 6, 1,
                Robot.CPin);
        // COUNTS_PER_REV    = 2048.0 ;    // CUI ATM103 Encoder at most PPR. Getting 4 times this.
        // WHEELDIA = 96.0; // goBilda Rhino wheels
        // TICKSPERMM = (8192)/(96*Math.PI) = 27.16244;
        // Yaw PID terms: kp 0.45, ki 0.12, kd 0.05
//        TWBController = new TwoWheelBalanceController(hardwareMap, 246.0,
//                27.16244, 0.45, 0.0, 0.05, 6, 1,
//                TwoWheelBalanceController.Robot.CPin);

        // These are the state terms for a two wheel balancing robot
        // Tune these using the DOE (Design of Experiments) opmode.
        // Both Kpos and Kvelo are negative when the center of mass is below the wheel axles
        // and positive when the CM is above (unstable). Sign does not change for Kpitch & KpitchRate
        //                            Kpos        Kvelo       Kpitch       KpitchRate
        setBalanceTerms(-0.01,-0.0025,-0.21,-0.0046);
        //                    -0.01       -0.0022       -0.21          -0.0044

        setArmPitchTarget(-0.5); // measure with C_Pitch_Fuzz opmode

        //TWBController.setDriveMotors(true,false,true); // REV IMU
        setDriveMotors(false,true,false); // Pinpoint

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
        this.updatePinpoint();
    }
    /**
     * Start is called once after play is pushed and calls the TWB controller start
     */
    public void start() {
        super.start();
        moveGearUp();
    }
    /**
     * TWB Main Loop method.  Call repeatedly while running. Contains balance control logic.
     * Teleoperated inputs are removed from this method, so it can be called in autonomous.
      */
    public void loopC(OpMode theOpmode) {

        if (!GearDown) {
            loop(theOpmode); // balancing
        } else { // gear is down or going down
            if (gearTimer.seconds() < 0.4) {
                loop(theOpmode); // keep balancing while going down
            } else {
                setMotorsZero();
            }
        }

        shoot_loop(); // check if we are shooting

        if (writeDatalog) {
            CdatalogTWB.logPosPitch(getPos() ,getPosTarget(),
                    getVelocity(), getVeloTarget(),getPitch(),
                    getPitchTarget(), getPitchRate(), getYaw(),getYawTarget(),
                    getPositionVolts(),getPitchVolts(), getDeltaTime());
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
        setYawTarget(getYawTarget() + deltaAngle );
    }

    /**
     * TWB method translates the robot at the current angle by setting Position, Velocity & Pitch Targets.
     * @param forward value from -1 to 1 that is the forward or backward amount
     * @param degPerLoop robot pitch degrees per loop, multiplied by forward (6 is good)
     * @param mmPerLoop robot translation in mm per loop, multiplied by forward (7 is good)
     */
    public void translateDrive(double forward, double mmPerLoop, double degPerLoop) {

        // add some pitch to get it moving (degPerLoop of 6 results in gentle movement)
        setAutoPitchTarget(forward * degPerLoop);

        // Update posTarget (mm) Note: this value * 50 = mm per second
        // mmPerLoop of 7 results in a gentle speed
        setPosTarget(getPosTarget() - forward * mmPerLoop );

        // Update the velocity target (mm/sec)
        //TWBController.setVeloTarget( -forward*(mmPerLoop/0.020));
        //TWBController.setVeloTarget( -forward*(mmPerLoop/TWBController.getDeltaTime()));
    }
    public void writeTelemetry(OpMode om) {
        om.telemetry.addLine(String.format(Locale.US, "s Position Target %.1f ,Current %.1f (mm)",
                getPosTarget(),getPos()));
        om.telemetry.addLine(String.format(Locale.US, "s Velocity Target %.1f ,Current %.1f (mm/sec)",
                getVeloTarget(),getVelocity()));
        om.telemetry.addLine(String.format(Locale.US, "Pitch Target %.1f ,Current %.1f (DEGREES)",
                getPitchTarget(),getPitch()));
        om.telemetry.addLine(String.format(Locale.US, "Yaw Target %.1f ,Current %.1f (RADIANS)",
                getYawTarget(),getYaw()));
        om.telemetry.addData("Left Ticks   ",getLeftTicks());
        om.telemetry.addData("Right Ticks   ",getRightTicks());
    }
    public void writeLog(String LogName) {
        writeDatalog=true;
        CdatalogTWB = new DatalogTWB();
        CdatalogTWB.init(LogName);
    }
    public void setMMPLoop(double mmpLoop) {MMPLoop = mmpLoop;}
    public void setDEGPLoop(double degpLoop) {DEGPLoop = degpLoop;}
    public double getMMPLoop() {return MMPLoop;}
    public double getDEGPLoop() {return DEGPLoop;}
}