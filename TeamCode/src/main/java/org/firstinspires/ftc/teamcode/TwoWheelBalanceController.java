
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Balance Controller class for a two wheel balancing robot.
 * Defines the motors and odometry (for position and velocity) for each wheel.
 * Uses four terms (states) to control balance: Position, Velocity, Pitch, PitchRate
 * Also provides Yaw (turn) control using a PID.
 * IMU provides pitch, pitchRate, Yaw and YawRate
 * The Position and velocity is at the robots center of mass (see vertCM & TWBOdometry).
 */
public class TwoWheelBalanceController {
    private final DcMotor leftDrive;
    private final DcMotor rightDrive;

    private final TWBOdometry odometry; // two wheel odometry object with running average
    private int leftTicks = 0;
    private int rightTicks = 0;
    private int leftZeroTicks = 0; // pinpoint does not reset.  have to store zero at start
    private int rightZeroTicks = 0; // pinpoint does not reset.  have to store zero at start

    // These are the state terms for a two wheel balancing robot
    private double Kpitch = -0.0001; // volts/degree
    private double KpitchRate = -0.0001; // volts/degrees/sec

    // Both Kpos and Kvelo are negative when the center of mass is below the wheel axles
    // and positive when the CM is above (unstable)
    private double Kpos = 0.0001;  // volts/mm
    private double Kvelo = 0.0001;  // volts/mm/sec

    private double TICKSPERMM = 0; // Encoder/drive/wheel Constant. set in initialization

    private boolean revEncoders = false; // reverse sign of encoders? Should be in child class

    // YAW PID
    private final PIDController yawPID;

    private double posTarget = 0.0;
    private double sOdom = 0.0; // Current robot position from odometry

    private double linearVelocity = 0.0;

    private double maxLinearVelocity = 0.0;  // (mm/sec) Calculated based on wheel dia and gear ratio
    private double maxAllowedVelocity = 1.0; // defines the maximum robot velocity
    private double maxAllowedAccel = 1.0; // maximum allowed robot acceleration
    private double vertCM = 10.0;  // vertical distance mm from the wheel center to the robot center of mass
    private double zeroPitchTarget = 0; // pitch of the imu when the robot is balanced upright (measure)
    private double addPitchTarget = 0; // for use when the robots CM changes in use, then the balance pitch changes
    private double pitchTarget = 0;  // The sum of the above two variables

    private double pitch = 0;  // degrees
    private double oldPitch = 0;
    private double pitchRATE = 0;

    private double yawTarget = 0.0;  // radians
    private double yaw = 0;  // radians
    private double priorYaw = 0;
    private double rawYaw = 0;
    private double rawPriorYaw = 0;
    private double yawRate = 0;

    public IMU imu; // Built-in REV IMU.  Should move to child class?

    private GoBildaPinpointDriver pinPoint; // goBilda Pinpoint Odometry Computer.  Should move to child class?

    private YawPitchRollAngles orientation;   // part of FIRST navigation classes

    private double positionVolts = 0.0;
    private double pitchVolts = 0.0;

    private double deltaTime = 0.02; // keeps the last loop time (seconds)
    private final ElapsedTime cycleTimer = new ElapsedTime();
    private double TARGET_LOOP_MS = 20.0; // Target 20ms (50Hz). Robot dependant?



    private DatalogTWB datalogTWB; // datalog for full recording
    private boolean writeDatalog = false; // default is no log.  call method to write.

    private final RunningAverageArray joystickS; // to smooth aggressive joystick inputs

    /**
     * TWB Constructor.  Call once in initialization.
     * Sign convention: L -^- R : + dist + velocity as shown.
     * Motors and Encoders both have sign!
     * Pitch: + pitch + pitch_rate is "nose" up.
     * Yaw:  L - +CCW - R  + yaw + yaw_rate is CCW from above.
     * @param hardwareMap hardware map
     * @param wheelBase Distance between Wheels in mm
     * @param ticksPerMM Odometry ticks per mm of wheel travel
     * @param kp Yaw PID Kp term
     * @param ki Yaw PID Ki term
     * @param kd Yaw PID Kd term
     * @param NVelo Size of running average array for robot velocity
     * @param NDist Size of running average array for robot odometry distance
      */
    public TwoWheelBalanceController(HardwareMap hardwareMap, double wheelBase,
                                     double ticksPerMM, double kp, double ki, double kd,
                                     int NVelo, int NDist) {

        // Define and Initialize Motors
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        imu = hardwareMap.get(IMU.class, "imu");

        odometry = new TWBOdometry(wheelBase, getPitch(),NVelo,NDist); // create odometry object
        TICKSPERMM = ticksPerMM;

        yawPID = new PIDController(kp, ki, kd);

        yawPID.setSetpoint(0.0);    // initial yaw (yawTarget) is zero.

        joystickS = new RunningAverageArray(40,true); // initialize size of running average
    }
    public void initializePinpoint(HardwareMap hardwareMap) {
        // initialize the Pinpoint, that has an IMU
        pinPoint = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        pinPoint.setOffsets(0.0, 0.0, DistanceUnit.MM);
        pinPoint.setEncoderResolution(27.16244, DistanceUnit.MM);
        pinPoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinPoint.resetPosAndIMU(); // recalibrates IMU
        leftZeroTicks = pinPoint.getEncoderX();
        rightZeroTicks = pinPoint.getEncoderY();
    }
    public void setDriveMotors(boolean leftForward, boolean rightForward, boolean reverseEncoders) {
        if(leftForward) leftDrive.setDirection(DcMotor.Direction.FORWARD);
        else leftDrive.setDirection(DcMotor.Direction.REVERSE);

        if(rightForward) rightDrive.setDirection(DcMotor.Direction.FORWARD);
        else rightDrive.setDirection(DcMotor.Direction.REVERSE);

        resetMotors();
        this.revEncoders = reverseEncoders;
    }

    /**
     * Sets Maximum Linear Velocity in mm/second
     * @param wheelDia in mm
     * @param gearRatio ratio from the motor to the wheels
     */
    public void setMaxLinearVelocity(double wheelDia, double gearRatio) {
        maxLinearVelocity = Math.PI * wheelDia * 100 / gearRatio;
    }

    /**
     * Returns the Maximum Robot Linear Velocity based on motors, wheels, gearing
     * @return the robot maximum linear velocity
     */
    public double getMaxLinearVelocity() {return maxLinearVelocity;}

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

        // reset the loop cycle timer
        cycleTimer.reset();

        // reset the PIDs
        yawPID.reset();
    }

    public void zeroPinpointTicks() {
        leftZeroTicks = pinPoint.getEncoderX();
        rightZeroTicks = pinPoint.getEncoderY();
    }
    private void resetMotors() {
        // reset the encoders
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // reset the motors
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** Call this at the beginning of loop()
     * This must be called at the start of the loop to get accurate loop cycle times.
     */
    public void startCycleTImer() {
        cycleTimer.reset();
    }
    /**
     * TWB Main Loop method.  Call repeatedly while running. Contains balance control logic.
     * Teleoperated inputs are removed from this method, so it can be called in autonomous.
      */
    public void loop(OpMode theOpmode) {

        updateTicks();

        updatePitchYaw();

        // update position and linear velocity values from wheel encoders (odometry)
        odometry.update(leftTicks / TICKSPERMM,
                rightTicks / TICKSPERMM, vertCM, pitch, deltaTime);
        sOdom = odometry.getS();  // position
        linearVelocity = odometry.getAvgLinearVelocity();

        // MAIN BALANCE CONTROL CODE:
        double posError = sOdom - posTarget;
        positionVolts = Kvelo * linearVelocity + Kpos * posError;

        pitchTarget = zeroPitchTarget + addPitchTarget;
        double pitchError = pitch - pitchTarget;

        pitchVolts = Kpitch * pitchError + KpitchRate * pitchRATE;
        double totalPowerVolts = pitchVolts + positionVolts;

        makeYawContinuous();

        yawPID.setSetpoint(yawTarget);
        double yawPower = yawPID.compute(yaw,yawRate);

         // Set the motor power for both wheels
        leftDrive.setPower(totalPowerVolts  - yawPower);
        rightDrive.setPower(totalPowerVolts  + yawPower);

        // kill the robot if it pitches over too far or runs fast when not asked to
        if ((Math.abs(pitch) > 28.0)  || (Math.abs(linearVelocity) > 1.3*maxLinearVelocity)) {
            theOpmode.requestOpModeStop(); // Stop the opmode
        }

        if (writeDatalog) {
            datalogTWB.logPosPitch(getPos(), odometry.getX(), odometry.getY(), getPosTarget(),
                    getVelocity(), getAcceleration(),getPitch(),
                    getPitchTarget(), getPitchRate(), getYaw(),getYawTarget(),
                    getPositionVolts(),getPitchVolts(), getDeltaTime());
            datalogTWB.writeLineTWB();
        }

        // Stall/wait out the rest of the target loop time
        while (cycleTimer.milliseconds() < TARGET_LOOP_MS) {
            // Yield thread slightly to prevent maxing out CPU completely
            Thread.yield();
        }
        deltaTime = cycleTimer.seconds(); // save last loop time for other processes
        //cycleTimer.reset();

    }
    public void makeYawContinuous() {
        // The following controls the turn (yaw) of the robot
        // IMU getYaw always returns value from -2*PI to 2*PI
        // The code below makes "yaw" a continuous value
        double deltaYaw = rawYaw - rawPriorYaw;
        rawPriorYaw = rawYaw;
        if (deltaYaw > Math.PI) deltaYaw -= 2 * Math.PI;
        else if (deltaYaw < -Math.PI) deltaYaw += 2 * Math.PI;
        yaw = priorYaw + deltaYaw;
        priorYaw = yaw;
    }
    public void setMotorsZero() {
        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
    }
    public void imuYawPitchReset() {
        // Doesn't seem to be working if robot yaw is greater than 180
        imu.resetYaw();
        yawTarget = 0.0;
        yaw = 0.0;
        priorYaw = 0.0;
        rawPriorYaw = 0.0;
        yawPID.reset();
        orientation = imu.getRobotYawPitchRollAngles();
        pitch = orientation.getPitch(AngleUnit.DEGREES);
    }

    public void updateTicks() {
        if (revEncoders) {
            leftTicks = -leftDrive.getCurrentPosition();
            rightTicks = -rightDrive.getCurrentPosition();
        } else {
            leftTicks = leftDrive.getCurrentPosition();
            rightTicks = rightDrive.getCurrentPosition();
        }
    }
    public void updateTicksPinpoint() {
        pinPoint.update(); // Update the pinpoint values for the following calls
        leftTicks = pinPoint.getEncoderX()-leftZeroTicks;
        rightTicks = pinPoint.getEncoderY()-rightZeroTicks;
    }
    public void updatePitchYaw() {
        // get pitch and pitch rate values from the IMU
        orientation = imu.getRobotYawPitchRollAngles();
        pitch = orientation.getPitch(AngleUnit.DEGREES);

        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        pitchRATE = angularVelocity.xRotationRate;

        rawYaw = orientation.getYaw(AngleUnit.RADIANS);
    }
    public void updatePitchYawPinpoint() {
        pitch = pinPoint.getPitch(AngleUnit.DEGREES);
        pitchRATE = (pitch- oldPitch)/deltaTime;
        oldPitch = pitch;

        yaw = -pinPoint.getHeading(UnnormalizedAngleUnit.RADIANS);
        yawRate = pinPoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
    }
    public void writeDatalog(String LogName) {
        this.writeDatalog=true;
        datalogTWB = new DatalogTWB();
        datalogTWB.init(LogName);
    }
    /**
     * TWB method to provide user control of turning the robot.
     * @param deltaAngle turn amount in radians
     */
    public void turn_teleop(double deltaAngle) {
        // Robot Turning:  turn the robot by adjusting the yaw PID setpoint (target)
        setYawTarget(getYawTarget() - deltaAngle );
    }

    /**
     * TWB method translates the robot by setting Position & Pitch Targets.
     *
     * @param gamepadStick value from -1 to 1 that is the forward or backward amount
     */
    public void translateDrive(double gamepadStick) {
        // Use running average of the joystick to smooth aggressive inputs.
        joystickS.add(gamepadStick);
        // Update posTarget (mm)
        setPosTarget( getPosTarget() - joystickS.getAverage() * maxAllowedVelocity * getDeltaTime() );
    }
    public void setTARGET_LOOP_MS(double targetLoopMs) {TARGET_LOOP_MS = targetLoopMs;}
    public double getDeltaTime() {return deltaTime;}
    public double getPitchTarget() {return pitchTarget;}
    public void setPosTarget(double pos) {posTarget = pos;}
    public double getPos() {return sOdom;}
    public double getPosTarget() {return posTarget;}
    public double getAcceleration() {return odometry.getAcceleration();}
    public double getVelocity() {return linearVelocity;}
    public void setAddPitchTarget(double target) { addPitchTarget = target;   }
    public void setZeroPitchTarget(double target) { zeroPitchTarget = target;   }

    public double getZeroPitchTarget() {return zeroPitchTarget;}
    public void setYawTarget(double yaw) { yawTarget = yaw; }
    public double getYawTarget() {return yawTarget;}
    public double getPositionVolts() { return positionVolts;}
    public double getPitchVolts() {return pitchVolts;}
    public double getKpitch() {return Kpitch;}
    public double getKpos() {return Kpos;}
    public double getKpitchRate() {return KpitchRate;}
    public double getKvelo() {return Kvelo;}
    public void setKpos(double k) {Kpos = k;}
    public void setKpitch(double k) {Kpitch = k;}
    public void setKpitchRate(double k) {KpitchRate=k;}
    public void setKvelo(double k) {Kvelo = k;}
    public double getYaw() {return yaw; }
    public double getPitch() { return pitch;}

    /**
     * This call is much slower than getPitch!
     * @return a recently read imu pitch value
     */
    public double getNewPitch() {
        // get values from the IMU.  Much slower than getPitch.
        orientation = imu.getRobotYawPitchRollAngles();
        pitch = orientation.getPitch(AngleUnit.DEGREES);
        return pitch;
    }
    public double getPitchRate() {return pitchRATE;}
    //public  int getLeftTicks() {return leftTicks;}
    //public  int getRightTicks() {return rightTicks;}
    public double getVerticalCM() {return vertCM;}
    public void setVerticalCM(double verticalCM) {vertCM = verticalCM;}
    public void setMaxAllowedVelocity(double maxVelo) {
        maxAllowedVelocity = maxVelo;}
    public double getMaxAllowedVelocity() {return maxAllowedVelocity;}
    public void setMaxAllowedAccel(double maxAccel) {
        maxAllowedAccel = maxAccel;
    }
    public double getMaxAllowedAccel() {return maxAllowedAccel;}
    public double getY() {return odometry.getY();}
}