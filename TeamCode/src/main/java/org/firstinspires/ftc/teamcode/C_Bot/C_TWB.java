package org.firstinspires.ftc.teamcode.C_Bot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TwoWheelBalanceController;

import java.util.Locale;

/**
 * Black Wheeled Two Wheel Balancing Robot Class
 *  Extends TWB class
 * Robot Details: goBilda 9.6 cm wheels, goBilda 26.9:1 motors, 1:1 belt drive
 * Using PinPoint
 */
public class C_TWB extends TwoWheelBalanceController {
    private boolean GearDown = true;
    //private final Servo leftGearServo;
    private final Servo rightGearServo;

    private final static double RIGHTDOWN = 0.09; // servo value
    private final static double RIGHTUP = 0.68;  // servo value
    //private final static double LEFTDOWN = 0.92; // servo value.  DETACHED
    //private final static double LEFTUP = 0.40;  // servo value   DETACHED
    public final static double GEARDOWNTIME = 0.55; // seconds to put the gear down
    private final ElapsedTime gearTimer = new ElapsedTime(); // Timer used with Claw

    private final DcMotor flywheel;
    private final ElapsedTime shotTimer = new ElapsedTime(); // Timer used for shooting
    private boolean shooting = false;
    private boolean collecting = false;
    /**
     * TWB Constructor.  Called once
      */
    public C_TWB(HardwareMap hardwareMap) {
        super(hardwareMap, 246.0,27.16244, 0.5, 0.0, 0.05, 6, 1);
        // kp was 0.45
        // COUNTS_PER_REV    = 2048.0  CUI ATM103 Encoder at most PPR. Getting 4 times this.
        // WHEELDIA = 96.0 mm goBilda Rhino wheels
        // TICKSPERMM = (8192)/(96*Math.PI) = 27.16244
        // Yaw PID terms: kp 0.45, ki 0.12, kd 0.05
        // goBilda 26.9:1 motors
        setMaxLinearVelocity(96.0, 26.9);

        initializePinpoint(hardwareMap); // includes the IMU

        // These are the state terms for a two wheel balancing robot
        // Tune these using the DOE (Design of Experiments) opmode.
        // Both Kpos and Kvelo are negative when the center of mass is below the wheel axles
        // and positive when the CM is above (unstable). Sign does not change for Kpitch & KpitchRate
        //                      Kpos        Kvelo       Kpitch       KpitchRate
        setBalanceTerms(-0.012,-0.0021,-0.20,-0.0044);
        //                    -0.01       -0.0022       -0.21          -0.0044

        setTARGET_LOOP_MS(20.0); // This has been tested and seems good

        setMaxAllowedVelocity(700.0);
        setMaxAllowedAccel(1000.0);

        setZeroPitchTarget(0.0); // zero angle, degrees, measure with PitchTune opmode

        setVerticalCM(130.0); // mm

        //TWBController.setDriveMotors(true,false,true); // REV IMU
        setDriveMotors(false,true,false); // Pinpoint

        //leftGearServo = hardwareMap.get(Servo.class, "leftGearServo");
        rightGearServo = hardwareMap.get(Servo.class, "rightGearServo");

        flywheel = hardwareMap.get(DcMotor.class, "fly");
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void init() {
        moveGearDown();
    }

    public void init_loop() {

        updateTicksPinpoint();
        updatePitchYawPinpoint();

    }
    /**
     * Start is called once after play is pushed and calls the TWB controller start
     */
    public void start() {
        super.start();
        moveGearUp();
        updateTicksPinpoint();
        zeroPinpointTicks();
    }
    /**
     * TWB Main Loop method.  Call repeatedly while running. Contains balance control logic.
     * Teleoperated inputs are removed from this method, so it can be called in autonomous.
      */
    public void loopC(OpMode theOpmode) {

        if (!GearDown) {
            loop(theOpmode); // balancing
        } else { // gear is down or going down
            if (gearTimer.seconds() < GEARDOWNTIME) {
                loop(theOpmode); // keep balancing while going down
            } else {
                setMotorsZero();
            }
        }

        shoot_loop(); // check if we are shooting
    }

    @Override
    public void makeYawContinuous() {
        // do nothing with pinpoint
    }
    @Override
    public void updateTicks() {
        updateTicksPinpoint();
    }

    @Override
    public void updatePitchYaw() {
        updatePitchYawPinpoint();
    }
    public void moveGearUp() {
        //leftGearServo.setPosition(LEFTUP);
        rightGearServo.setPosition(RIGHTUP);
        GearDown = false;
    }
    public void moveGearDown() {
        // put the gear down and wait for a bit
        //leftGearServo.setPosition(LEFTDOWN);
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

    public void setMaxSpeedGamepad(OpMode om) {

        double maxVelo = getMaxAllowedVelocity();

        if (om.gamepad1.dpadUpWasPressed()) maxVelo += 10.0;
        else if (om.gamepad1.dpadDownWasPressed()) maxVelo -= 10.0;

        if (maxVelo < 20.0) maxVelo = 20.0;
        else if (maxVelo > 0.8*getMaxLinearVelocity()) maxVelo = 0.8*getMaxLinearVelocity();

        setMaxAllowedVelocity(maxVelo);

        om.telemetry.addLine("DPAD UP - DOWN Adjusts the Maximum Velocity");
        om.telemetry.addLine(" --- ");
        om.telemetry.addData("Max MOVE Velocity (mm/sec)"," %.1f", maxVelo);
        om.telemetry.addData("Max ROBOT Velocity (mm/sec)"," %.1f", getMaxLinearVelocity());
    }
    public void writeTelemetry(OpMode om) {
        om.telemetry.addLine(String.format(Locale.US, "s Position Target %.0f ,Current %.0f (mm)",
                getPosTarget(),getPos()));
        om.telemetry.addLine(String.format(Locale.US, "s Velocity Target %.0f ,Current %.0f (mm/sec)",
                getAcceleration(),getVelocity()));
        om.telemetry.addLine(String.format(Locale.US, "Pitch Target %.1f ,Current %.1f (DEGREES)",
                getPitchTarget(),getPitch()));
        om.telemetry.addLine(String.format(Locale.US, "Yaw Target %.1f ,Current %.1f (RADIANS)",
                getYawTarget(),getYaw()));
        //om.telemetry.addData("Left Ticks   ",getLeftTicks());
        //om.telemetry.addData("Right Ticks   ",getRightTicks());
    }

}