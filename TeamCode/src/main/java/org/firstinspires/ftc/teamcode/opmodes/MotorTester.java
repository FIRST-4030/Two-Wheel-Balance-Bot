package org.firstinspires.ftc.teamcode.opmodes;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * MotorTester.  Also checks the IMU
 *
 * @author Steve Amorori, 4/15/2026
 */
@TeleOp(name = "MotorTester", group="Util")
@Disabled
public class MotorTester extends OpMode {

    String DEVICE_NAME = "motor"; // plug motor and encoder into port 3 and name "motor"

    DcMotor motor;
    double power = 0.0;
    double increment = 0.02;
    boolean forward = true;

    IMU imu;
    YawPitchRollAngles orientation;   // part of FIRST navigation classes

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, DEVICE_NAME);
        motor.setDirection(DcMotor.Direction.FORWARD);
        // reset the encoder
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set to "run to power" mode
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // initialize IMU before odometry, because odometry needs pitch
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));
        imu.resetYaw(); // set the yaw value to zero
    }

    @SuppressLint("DefaultLocale")
    public void init_loop() {
        if (gamepad1.dpadUpWasReleased()) {
            power += increment;
        }

        if (gamepad1.dpadDownWasReleased()) {
            power -= increment;
        }
        telemetry.addLine("Plug motor and encoder into port 3");
        telemetry.addData("Ticks", motor.getCurrentPosition());
        telemetry.addLine("SPIN MOTOR BY HAND");

        orientation = imu.getRobotYawPitchRollAngles();
        telemetry.addData("Pitch ", "%.1f",orientation.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Yaw ", "%.1f",orientation.getYaw(AngleUnit.DEGREES));

        telemetry.update();
    }

    @SuppressLint("DefaultLocale")
    public void loop() {
        telemetry.addLine(String.format("DPad Left: Decrease Power by %.2f",increment));
        telemetry.addLine(String.format("DPad Right: Increase Power by %.2f",increment));
        telemetry.addData("Power", "%.2f", power);
        telemetry.addData("Ticks", motor.getCurrentPosition());

        if (gamepad1.dpadLeftWasPressed()) {
            power -= increment;
            power = Math.max(-1.0, power);
            motor.setPower(power);
        }

        if (gamepad1.dpadRightWasPressed()) {
            power += increment;
            power = Math.min(1.0, power);
            motor.setPower(power);
        }

        telemetry.addLine(String.format("A button toggles direction, forward = %B",forward));

        if (gamepad1.aWasPressed()) {
            power = 0.0;
            motor.setPower(power);
            if (forward) {
                motor.setDirection(DcMotor.Direction.REVERSE);
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                forward = false;
            } else {
                motor.setDirection(DcMotor.Direction.FORWARD);
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                forward = true;
            }

        }

        telemetry.update();
    }

}