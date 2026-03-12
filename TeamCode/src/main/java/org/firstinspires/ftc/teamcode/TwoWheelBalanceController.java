
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Balance Controller class for a two wheel balancing robot.
 * Defines the motors and odometry (for position and velocity) for each wheel.
 * Uses four terms (states) to control balance: Position, Velocity, Pitch, PitchRate
 * Also provides Yaw (turn) control using a PID.
 * IMU provides pitch, pitchRate, Yaw and YawRate
 * The Position and velocity is of the center of the wheels.
 */
public class TwoWheelBalanceController {
    private final DcMotor leftDrive;
    private final DcMotor rightDrive;

    private final TWBOdometry odometry; // two wheel odometry object with running average

    // These are the state terms for a two wheel balancing robot
    private double Kpitch = -0.0001; // volts/degree
    private double KpitchRate = -0.0001; // volts/degrees/sec
    // Have had difficulty tuning KpitchRate manually.  Use DOE opmode

    // Both Kpos and Kvelo are negative when the center of mass is below the wheel axles
    // and positive when the CM is above (unstable)
    private double Kpos = 0.0001;  // volts/mm
    private double Kvelo = 0.0001;  // volts/mm/sec

    private double TICKSPERMM = 1; // set in initialization

    // YAW PID
    private final PIDController yawPID;

    private double posTarget = 0.0;
    private double sOdom = 0.0; // Current robot position from odometry

    private double veloTarget = 0.0;
    private double linearVelocity = 0.0;
    private double autoPitchTarget = 0; // used to set pitch from an auto routine
    private double armPitchTarget = 0;
    private double pitchTarget = 0;

    private double pitch = 0; // degrees, value replaced with that from imu

    private double yawTarget = 0.0;
    private double yaw = 0;
    private double priorYaw = 0;
    private double rawPriorYaw = 0;

    private final IMU imu;
    private YawPitchRollAngles orientation;   // part of FIRST navigation classes

    private double currentVoltage = 12.0; // This value is overridden with a measured value in start()
    private double positionVolts = 0.0;
    private double pitchVolts = 0.0;

    private final ElapsedTime runtime = new ElapsedTime(); // Timer used to check loop times
    // The variables below are to try to get a consistent delta time for the controller.
    // Not sure how well this works. Don't know how to make it better without different runtime env.
    private final RunningAverage deltaTimeRA = new RunningAverage(6);
    private double currentTime;
    private double lastTime;
    private double deltaTime = 0.04; // initialize, replaced by a running average

    /**
     * TWB Constructor.  Call once in initialization.
     * @param hardwareMap hardware map
     * @param wheelBase Distance between Wheels in mm
     * @param wheelDia Wheel Diameter in mm
     * @param ticksPerMM Odometry ticks per mm of wheel travel
     * @param kp Yaw PID Kp term
     * @param ki Yaw PID Ki term
     * @param kd Yaw PID Kd term
      */
    public TwoWheelBalanceController(HardwareMap hardwareMap, double wheelBase, double wheelDia,double ticksPerMM,
                                     double kp, double ki, double kd) {

        deltaTimeRA.addNumber(0.04); // add to running average to smooth the start??

        // Define and Initialize Motors
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        resetMotors();

        odometry = new TWBOdometry(wheelBase, wheelDia, 0); // create odometry object
        TICKSPERMM = ticksPerMM;

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));

        yawPID = new PIDController(kp, ki, kd); // kp 0.45, ki 0.12, kd 0.05

        yawPID.setSetpoint(0.0);    // initial yaw (yawTarget) is zero.
    }

    /**
     * setBalanceTerms initializes the four balance controller terms
     * @param kpos K Position  volts/mm
     * @param kvelo K Velocity volts/mm/second
     * @param kpitch K Pitch   volts/degree
     * @param kpitchrate K Pitch Rate  volts/degree/second
     */
    public void setBalanceTerms(double kpos, double kvelo, double kpitch, double kpitchrate) {
        Kpos = kpos;
        Kvelo = kvelo;
        Kpitch = kpitch;
        KpitchRate = kpitchrate;
    }

    /**
     * TWB start method. Called once on Start press. Resets encoders, timers, PIDs
      */
    public void start() {
        resetMotors();

        orientation = imu.getRobotYawPitchRollAngles();
        pitch = orientation.getPitch(AngleUnit.DEGREES);
        imu.resetYaw(); // set the yaw value to zero

        // reset the timer
        runtime.reset();
        currentTime = runtime.seconds();
        lastTime = currentTime;

        // reset the PIDs
        yawPID.reset();
    }
    /**
     *  Return the pitch from the IMU
     */
    public double getPitch() {
        orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getPitch(AngleUnit.DEGREES);
    }

    private void resetMotors() {
        // reset the encoders
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // reset the motors
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    /**
     * TWB Main Loop method.  Call repeatedly while running. Contains balance control logic.
     * Teleoperated inputs are removed from this method, so it can be called in autonomous.
      */
    public void loop(OpMode theOpmode) {
        setLoopTime(); // this updates the deltaTime value

        double leftTicks = leftDrive.getCurrentPosition();
        double rightTicks = rightDrive.getCurrentPosition();

        AngularVelocity angularVelocity;  // part of FIRST navigation classes

        // get values from the IMU
        orientation = imu.getRobotYawPitchRollAngles();
        //yaw = orientation.getYaw(AngleUnit.DEGREES);
        angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        pitch = orientation.getPitch(AngleUnit.DEGREES);
        double pitchRATE = angularVelocity.xRotationRate;

        // get values from wheel encoders (odometry)
        odometry.update(leftTicks / TICKSPERMM, rightTicks / TICKSPERMM, pitch, deltaTime);
        sOdom = odometry.getS();
        linearVelocity = odometry.getAvgLinearVelocity();

        // MAIN BALANCE CONTROL CODE:
        double posError = sOdom - posTarget;
        double veloError = linearVelocity + veloTarget;
        positionVolts = Kvelo * veloError + Kpos * posError;

        pitchTarget = armPitchTarget + autoPitchTarget;
        double pitchError = pitch - pitchTarget;

        pitchVolts = Kpitch * pitchError + KpitchRate * pitchRATE;
        double totalPowerVolts = pitchVolts + positionVolts;

        // The following controls the turn (yaw) of the robot
        // getYaw always returns value from -2*PI to 2*PI
        double rawYaw = orientation.getYaw(AngleUnit.RADIANS);
        // The code below makes "yaw" a continuous value TO DO: MAKE THIS A METHOD
        double deltaYaw = rawYaw - rawPriorYaw;
        rawPriorYaw = rawYaw;
        if (deltaYaw > Math.PI) deltaYaw -= 2 * Math.PI;
        else if (deltaYaw < -Math.PI) deltaYaw += 2 * Math.PI;
        yaw = priorYaw + deltaYaw;
        priorYaw = yaw;

        yawPID.setSetpoint(yawTarget);
        double yawPower = yawPID.compute(yaw);

        // limit the total volts
        if (totalPowerVolts > 14) totalPowerVolts = 14;
        else if (totalPowerVolts < -14) totalPowerVolts = -14;

         // Set the motor power for both wheels
        leftDrive.setPower(totalPowerVolts / currentVoltage - yawPower);
        rightDrive.setPower(totalPowerVolts / currentVoltage + yawPower);

        // kill the robot if it pitches over too far or runs fast when not asked to
        if (Math.abs(pitch) > 90  || (Math.abs(linearVelocity) > 1500 && Math.abs(veloTarget) < 50)) {
            theOpmode.requestOpModeStop(); // Stop the opmode
        }
    }
    private void setLoopTime () {
        // compute a loop time.  Using running average to smooth values
        lastTime = currentTime;
        currentTime = runtime.seconds();
        deltaTime = currentTime - lastTime;
        if(deltaTime > 0.05) deltaTime = 0.05; // fake!
        // add the new delta time to the running average
        deltaTimeRA.addNumber(deltaTime);
        deltaTime = deltaTimeRA.getAverage(); // replace deltaTime with running average delta time
    }
    public double getDeltaTime() {return deltaTime;}
    public double getPitchTarget() {return pitchTarget;}
    public void setPosTarget(double pos) {posTarget = pos;}
    public double getPosition() {return sOdom;}
    public double getPosTarget() {return posTarget;}

    public void setVeloTarget(double velo) {veloTarget = velo;}
    public double getVeloTarget() {return veloTarget;}
    public double getVelocity() {return linearVelocity;}
    public void setArmPitchTarget (double target) { armPitchTarget = target;   }
    public void setAutoPitchTarget (double target) { autoPitchTarget = target;   }
    public void setCurrentVoltage(double cv) { currentVoltage = cv;  }
    public void setYawTarget(double yaw) { yawTarget = yaw; }
    public double getYawTarget() {return yawTarget;}
    public double getPositionVolts() { return positionVolts;}
    public double getPitchVolts() {return pitchVolts;}
}