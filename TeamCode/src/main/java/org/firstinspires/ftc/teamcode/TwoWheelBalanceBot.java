/*
This class is for a Two Wheel Balancing Robot With Arm
*/

package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Two Wheel Balancing Robot Class, with Arm.
 * This class has Many members (variables)!
 */
public class TwoWheelBalanceBot {
    final private OpMode theOpmode; // Set during construction.  Enables using telemetry and gamepad

    public boolean TELEMETRY = true; // switch for telemetry

    // These are the state terms for a two wheel balancing robot
    public double Kpitch = -0.57; // volts/degree at arm = -90
    public double KpitchRate = -0.022; // volts/degrees/sec
    // Have had difficulty tuning this term.  Can't tell what changes it makes.

    public double Kpos = 0.017;  // volts/mm For high balancing (unstable) this term is positive
    public double Kvelo = 0.015;  // volts/mm/sec For high balancing (unstable) this term is positive
    // Larger Kvelo decreases the rocking motion, up to a point, then chatter!
    // Both Kpos and Kvelo are negative when the center of mass is below the wheel axles.

    static final double WHEELBASE = 300; // robot Wheel base (mm)

    static final double REVSPUR40PPR = 1120; // REV Core Hex Motor Pulses per Revolution at output shaft
    //static final double     COUNTS_PER_REV    = 2048.0 ;    // CUI ATM103 Encoder at most PPR
    static final double WHEELDIA = 203.0; // 8 inch wheel diameter (mm)

    static final double TICKSPERMM = (REVSPUR40PPR)/(WHEELDIA*Math.PI); // REV SPUR 40:1, 8in wheels

    // YAW PID
    PIDController yawPID = new PIDController(0.45, 0.14, 0.06); // kp 0.45, ki 0.12, kd 0.05

    public double posTarget = 0.0;
    public double veloTarget = 0.0;
    public double autoPitchTarget = 0; // used to set pitch from an auto routine
    double pitchTarget = 0;

    double pitch = 0; // degrees, value got from imu
    double pitchError = 0;

    public double yawTarget = 0.0; // from the user joystick in teleop or from auto routines
    public double yaw = 0;
    double priorYaw = 0;
    double rawYaw, rawPriorYaw = 0;
    double yawPower;

    public boolean ClawIsClosed = false; //Claw boolean

    final private IMU imu;

    private YawPitchRollAngles orientation;   // part of FIRST navigation classes

    final private VoltageSensor battery;
    double currentVoltage = 12.0; // This value is overridden with a measured value in start()
    public double positionVolts = 0.0;
    public double pitchVolts = 0.0;

    //Handles the arm control, and adjusting the arm for the pitch of the robot
    final public TWBArmServo theArm;

    public DcMotor leftDrive;
    public DcMotor rightDrive;

    TWBOdometry odometry; // two wheel odometry object with running average
    int i = 0;  // loop counter, used with data logging

    ElapsedTime runtime = new ElapsedTime(); // Timer used to check loop times
    private final RunningAverage deltaTimeRA = new RunningAverage(8);
    private double currentTime;

    public double sOdom; // Current robot position from odometry
    double theta;  // used with odometry

    double armPitchTarget = 0; // degrees

    public Servo clawServo;

    /**
     * TWB Constructor.  Call once in initialization.
      */
    public TwoWheelBalanceBot(HardwareMap hardwareMap, OpMode opMode) {

        this.theOpmode = opMode; // set the opmode that is calling this class

        deltaTimeRA.addNumber(0.04); // add to running average to smooth the start

        // Define and Initialize Motors
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        odometry = new TWBOdometry(WHEELBASE, WHEELDIA, pitch); // create odometry object

        // Get devices from the hardwareMap.
        // as needed, change "Control Hub" to (e.g.) "Expansion Hub 1".
        battery = hardwareMap.voltageSensor.get("Control Hub");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));

        // Initialize the arm class
        // ARM LIMITS ARE DEFINED IN ArmServoTWB class
        theArm = new TWBArmServo(hardwareMap, "arm_servo", 0.0, 140, -165, 60);
        // NOTE: Set arm angle to zero to rig the servo (so it is easy to see)

        clawServo = hardwareMap.get(Servo.class, "clawServo");

        yawPID.setSetpoint(0.0);    // initial yaw (yawTarget) is zero.

    }

    /**
     * TWB init. Called once at initialization
     */
    public void init() {
    }
    /**
     * TWB init loop.  Called repeatedly in initialization.
      */
    public void init_loop() {
        //theOpmode.telemetry.addData("LOG", LOG);
    }

    /**
     *  TWB automatic self righting method.  Call repeatedly in initialization.
      */
    public void auto_right_loop() {
        theOpmode.telemetry.addData("AUTO-RIGHT", "ACTIVE");
        orientation = imu.getRobotYawPitchRollAngles();
        pitch = orientation.getPitch(AngleUnit.DEGREES);
        theOpmode.telemetry.addData("Pitch (DEG)", pitch);

        // Check which way the robot is leaning and rotate the arm so that it will self-right
        if (pitch > 0.0) theArm.setArmAngle(140.0);
        else theArm.setArmAngle(-150.0);
        theOpmode.telemetry.addData("Arm Angle (DEG)", theArm.getAngle());
        armPitchTarget = theArm.updateArm(0.02); // This will make the arm move

        if (ClawIsClosed) clawServo.setPosition(1.0); // closed value (0.98 for blocks)
        else clawServo.setPosition(0.35); // open value (WAS 0.35)

    }

    /**
     * TWB start method. Called once on Start press. Resets encoders, timers, PIDs
      */
    public void start(double armAngle) {
        // reset the encoders because the robot may have moved during auto righting
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        orientation = imu.getRobotYawPitchRollAngles();
        pitch = orientation.getPitch(AngleUnit.DEGREES);
        imu.resetYaw(); // set the yaw value to zero

        // reset the timers for the datalog timestamp, turns, runToPos
        runtime.reset();

        // reset the PIDs
        yawPID.reset();

        // Get the current voltage, so that balance control is more consistent
        currentVoltage = battery.getVoltage();
        currentTime = runtime.seconds(); // initialize current time for delta time values

        theArm.setArmAngle(armAngle);
    }

    /**
     * TWB Main Loop method.  Call repeatedly while running. Contains balance control logic.
     * Teleoperated inputs are removed from this method, so it can be called in autonomous.
      */
    @SuppressLint("DefaultLocale")
    public void loop() {
        double leftTicks = leftDrive.getCurrentPosition();
        double rightTicks = rightDrive.getCurrentPosition();

        AngularVelocity angularVelocity;  // part of FIRST navigation classes

        i++;  // index the loop counter

        // compute a loop time.  Using running average to smooth values
        double lastTime = currentTime;
        currentTime = runtime.seconds();
        double deltaTime = currentTime - lastTime;
        // add the new delta time to the running average
        deltaTimeRA.addNumber(deltaTime);
        deltaTime = deltaTimeRA.getAverage(); // replace deltaTime with running average delta time

        // get values from the IMU
        orientation = imu.getRobotYawPitchRollAngles();
        //yaw = orientation.getYaw(AngleUnit.DEGREES);
        angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        pitch = orientation.getPitch(AngleUnit.DEGREES);
        double pitchRATE = angularVelocity.xRotationRate;

        // get values from wheel encoders (odometry)
        odometry.update(leftTicks / TICKSPERMM, rightTicks / TICKSPERMM, pitch, deltaTime);
        double linearVelocity = odometry.getAvgLinearVelocity();
        sOdom = odometry.getS();
        theta = odometry.getTheta(); // should be the same value as imu yaw

        // The robot pitch target is determined by the angle of the arm.
        armPitchTarget = theArm.updateArm(deltaTime); // update arm position (this makes it move)

        // MAIN BALANCE CONTROL CODE:
        double posError = sOdom - posTarget;
        double veloError = linearVelocity + veloTarget;
        positionVolts = Kvelo * veloError + Kpos * posError;

        pitchTarget = armPitchTarget + autoPitchTarget;
        pitchError = pitch - pitchTarget;

        pitchVolts = Kpitch * pitchError + KpitchRate * pitchRATE;
        double totalPowerVolts = pitchVolts + positionVolts;

        // The following controls the turn (yaw) of the robot
        // getYaw always returns value from -2*PI to 2*PI
        rawYaw = orientation.getYaw(AngleUnit.RADIANS);
        // The code below makes "yaw" a continuous value TO DO: MAKE THIS A METHOD
        double deltaYaw = rawYaw - rawPriorYaw;
        rawPriorYaw = rawYaw;
        if (deltaYaw > Math.PI) deltaYaw -= 2 * Math.PI;
        else if (deltaYaw < -Math.PI) deltaYaw += 2 * Math.PI;
        yaw = priorYaw + deltaYaw;
        priorYaw = yaw;

        yawPID.setSetpoint(yawTarget);
        yawPower = yawPID.compute(yaw);

        // limit the total volts
        if (totalPowerVolts > 14) totalPowerVolts = 14;
        else if (totalPowerVolts < -14) totalPowerVolts = -14;

         // Set the motor power for both wheels
        leftDrive.setPower(totalPowerVolts / currentVoltage - yawPower);
        rightDrive.setPower(totalPowerVolts / currentVoltage + yawPower);

        if (ClawIsClosed) clawServo.setPosition(1.0); // closed value (0.98 for blocks)
        else clawServo.setPosition(0.35); // open value (WAS 0.35)

        if (TELEMETRY) {
            theOpmode.telemetry.addLine(String.format("s Position Target %.1f ,Current %.1f (mm)",posTarget,sOdom));
            theOpmode.telemetry.addLine(String.format("s Velocity Target %.1f ,Current %.1f (mm/sec)",veloTarget,linearVelocity));
            theOpmode.telemetry.addLine(String.format("Pitch Target %.1f ,Current %.1f (degrees)",pitchTarget,pitch));
            theOpmode.telemetry.addLine(String.format("Arm Angle Target %.1f ,Current %.1f (degrees)",theArm.getTargetAngle(),theArm.getAngle()));
            theOpmode.telemetry.addData("Claw Servo", clawServo.getPosition());
        }

        // kill the robot if it pitches over too far or runs fast when not asked to
        if (Math.abs(pitch) > 80  || (Math.abs(linearVelocity) > 1500 && Math.abs(veloTarget) < 50)) {
            theOpmode.requestOpModeStop(); // Stop the opmode
        }
    }


    /**
     * TWB method to provide buttons for tuning feedback constants.
     */
    public void tuneButtons() {
        if (theOpmode.gamepad1.dpadUpWasPressed()) Kpitch += 0.01;
        else if (theOpmode.gamepad1.dpadDownWasPressed()) Kpitch -= 0.01;
        else if (theOpmode.gamepad1.dpadLeftWasPressed()) Kvelo += 0.001;
        else if (theOpmode.gamepad1.dpadRightWasPressed()) Kvelo -= 0.001;
        else if (theOpmode.gamepad1.yWasPressed()) KpitchRate += 0.001;
        else if (theOpmode.gamepad1.aWasPressed()) KpitchRate -= 0.001;
        else if (theOpmode.gamepad1.xWasPressed()) Kpos += 0.001;
        else if (theOpmode.gamepad1.bWasPressed()) Kpos -= 0.001;

        theOpmode.telemetry.addData("K pos   +X - B", Kpos);
        theOpmode.telemetry.addData("K velo  +LEFT -RIGHT", Kvelo);
        theOpmode.telemetry.addData("K pitch +UP -DOWN", Kpitch);
        theOpmode.telemetry.addData("K pitch rate +Y -A", KpitchRate);
    }

    /**
     * TWB method to provide user control of the arm.
     */
    public void arm_teleop() {
        //Increment target Arm angle
        if (theOpmode.gamepad1.right_stick_y > 0.3) theArm.setArmAngle(theArm.getAngle() + 2.0);
        else if (theOpmode.gamepad1.right_stick_y < -0.3) theArm.setArmAngle(theArm.getAngle() - 2.0);

        //Set arm angle for cargo deposit
        if (theOpmode.gamepad1.left_bumper) theArm.setArmAngle(-90.0);

        //Set arm angle to cargo collection
        if (theOpmode.gamepad1.right_trigger_pressed) theArm.setArmAngle(-150.0);

        //Sets arm to vertical. Increment and hold are modified to properly represent the operative state
        if (theOpmode.gamepad1.left_trigger > 0.5) theArm.setArmAngle(0.0);
    }

    /**
     * TWB method to provide user control of the claw.
     */
    public void claw_teleop() {
        //Controls the claw boolean
        if (theOpmode.gamepad1.rightBumperWasPressed()) {
            if (ClawIsClosed)  ClawIsClosed = false; // open
            else { // close
                theArm.setArmAngle(theArm.getAngle() + 15.0);  // raise the arm a bit
                ClawIsClosed = true;
            }
        }
    }

    /**
     * TWB method to provide user control of turning the robot.
     * @param deltaAngle a value from -1 to 1 in radians
     * @param scale a value to scale the deltaAngle
     */
    public void turn_teleop(double deltaAngle, double scale) {
        // Robot Turning:
        // The right joystick turns the robot by adjusting the yaw PID turn setpoint
        yawTarget -= deltaAngle * scale;  // get turn from gamepad (radian)
    }

    /**
     * TWB method to provide user control of moving the robot with joystick, controlling pitch.
     * @param forward value from -1 to 1 that is the forward or backward amount
     * @param degPerLoop is robot pitch degrees per loop, multiplied by forward (6 is good)
     * @param mmPerLoop is robot translation in mm per loop, multiplied by forward (7 is good)
     *                  degPerLoop and mmPerLoop should be close in value.
     */
    public void translateDrive(double forward, double mmPerLoop, double degPerLoop) {

        // add some pitch to get it moving
        autoPitchTarget = forward * degPerLoop; // degPerLoop of 6 results in gentle movement

        // Update posTarget (mm) Note: this value * 50 = mm per second
        posTarget -= forward * mmPerLoop; // mmPerLoop of 7 results in a gentle speed
    }

    public void imuYawReset() {
        // Doesn't seem to be working if robot yaw is greater than 180
        imu.resetYaw();
        yawTarget = 0.0;
        yaw = 0.0;
        yawPID.reset();

    }

}