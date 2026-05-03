package org.firstinspires.ftc.teamcode.opmodes;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.rev.Rev9AxisImuOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Simple drive opmode to test the C TWB robot.  This NOT use the C_TWB class. Does NOT balance.
 *
 * @author Steve Amorori, 4/15/2026
 */
@TeleOp(name = "C_SimpleDrive")
//@Disabled
public class C_DriveSimple extends OpMode {

    DcMotor leftmotor;
    DcMotor rightmotor;

    double power = 0.0;
    double increment = 0.02;

    IMU imu;
    IMU imuRev;
    YawPitchRollAngles orientation;   // part of FIRST navigation classes

    @Override
    public void init() {
        leftmotor = hardwareMap.get(DcMotor.class, "left_drive");
        leftmotor.setDirection(DcMotor.Direction.FORWARD);
        leftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightmotor = hardwareMap.get(DcMotor.class, "right_drive");
        rightmotor.setDirection(DcMotor.Direction.REVERSE);
        rightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // initialize IMU before odometry, because odometry needs pitch
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));
        imu.resetYaw(); // set the yaw value to zero

        imuRev = hardwareMap.get(IMU.class, "imuREV");
        imuRev.initialize(new IMU.Parameters(new Rev9AxisImuOrientationOnRobot(
                Rev9AxisImuOrientationOnRobot.LogoFacingDirection.UP,
                Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection.FORWARD)));
        imuRev.resetYaw();
    }

    @SuppressLint("DefaultLocale")
    public void init_loop() {

        orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        telemetry.addData("Pitch ", "%.1f",orientation.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Pitch Rate ", "%.1f",angularVelocity.xRotationRate);
        telemetry.addData("Yaw ", "%.1f",orientation.getYaw(AngleUnit.DEGREES));

        telemetry.addLine(" ---");

        YawPitchRollAngles orientationRev = imuRev.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocityRev = imuRev.getRobotAngularVelocity(AngleUnit.DEGREES);
        telemetry.addData("Pitch REV", "%.1f",orientationRev.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Pitch Rate REV", "%.1f",angularVelocityRev.xRotationRate);
        telemetry.addData("Yaw REV", "%.1f",orientationRev.getYaw(AngleUnit.DEGREES));

        telemetry.update();
    }

    @SuppressLint("DefaultLocale")
    public void loop() {
        telemetry.addLine(String.format("DPad Left: Decrease Power by %.2f",increment));
        telemetry.addLine(String.format("DPad Right: Increase Power by %.2f",increment));
        telemetry.addData("Power", "%.2f", power);
        telemetry.addData("Left Ticks", leftmotor.getCurrentPosition());
        telemetry.addData("Right Ticks", rightmotor.getCurrentPosition());

        if (gamepad1.dpadLeftWasPressed()) {
            power -= increment;
            power = Math.max(-1.0, power);
            leftmotor.setPower(power);
            rightmotor.setPower(power);

        }

        if (gamepad1.dpadRightWasPressed()) {
            power += increment;
            power = Math.min(1.0, power);
            leftmotor.setPower(power);
            rightmotor.setPower(power);
        }

        telemetry.update();
    }

}