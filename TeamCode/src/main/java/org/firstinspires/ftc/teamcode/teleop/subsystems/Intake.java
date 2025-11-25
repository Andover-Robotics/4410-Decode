package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake {

    public static double intakePower = -1, storagePower = -0.15, reversePower = 0.8, gateOpen = 0.1, gateClosed = 0.24;
    private final MotorEx motor;
    public Servo gate;

    private final DigitalChannel bottomBB;
    private final DigitalChannel middleBB;
    private final DigitalChannel topBB;

    //pin0 = green
    //pin1 = purple

    private final DigitalChannel cs1_pin0;
    private final DigitalChannel cs1_pin1;

    private final DigitalChannel cs2_pin0;
    private final DigitalChannel cs2_pin1;

    private final DigitalChannel cs3_pin0;
    private final DigitalChannel cs3_pin1;

    public Intake(OpMode opMode) {
        motor = new MotorEx(opMode.hardwareMap, "intake", Motor.GoBILDA.RPM_1150);
        motor.setInverted(false);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        gate = opMode.hardwareMap.servo.get("gate");

        bottomBB = opMode.hardwareMap.get(DigitalChannel.class, "bottomBB");
        middleBB = opMode.hardwareMap.get(DigitalChannel.class, "middleBB");
        topBB = opMode.hardwareMap.get(DigitalChannel.class, "topBB");

        cs1_pin0 = opMode.hardwareMap.get(DigitalChannel.class, "cs1_pin0");
        cs1_pin1 = opMode.hardwareMap.get(DigitalChannel.class, "cs1_pin1");

        cs2_pin0 = opMode.hardwareMap.get(DigitalChannel.class, "cs2_pin0");
        cs2_pin1 = opMode.hardwareMap.get(DigitalChannel.class, "cs2_pin1");

        cs3_pin0 = opMode.hardwareMap.get(DigitalChannel.class, "cs3_pin0");
        cs3_pin1 = opMode.hardwareMap.get(DigitalChannel.class, "cs3_pin1");

        cs1_pin0.setMode(DigitalChannel.Mode.INPUT);
        cs1_pin1.setMode(DigitalChannel.Mode.INPUT);

        cs2_pin0.setMode(DigitalChannel.Mode.INPUT);
        cs2_pin1.setMode(DigitalChannel.Mode.INPUT);

        cs3_pin0.setMode(DigitalChannel.Mode.INPUT);
        cs3_pin1.setMode(DigitalChannel.Mode.INPUT);
    }

    public void intake() {
        motor.set(intakePower);
    }

    public void storage() {
        motor.set(storagePower);
    }

    public void reverse() {
        motor.set(reversePower);
    }

    public void stop() {
        motor.set(0);
    }

    public void setPower(double power) {
        motor.set(power);
    }

    public void openGate() {
        gate.setPosition(gateOpen);
    }

    public void closeGate() {
        gate.setPosition(gateClosed);
    }

    public boolean holdingBottom() {
        return !bottomBB.getState();
    }

    public boolean holdingMiddle() {
        return !middleBB.getState();
    }

    public boolean holdingTop() {
        return !topBB.getState();
    }

    public int storageCount() {
        return (holdingBottom()? 1 : 0) + (holdingMiddle()? 1 : 0) + (holdingTop()? 1 : 0);
    }

    public boolean cs1_pin0Active() {
        return cs1_pin0.getState();
    }
    public boolean cs1_pin1Active() {
        return cs1_pin1.getState();
    }

    public boolean cs2_pin0Active() {
        return cs2_pin0.getState();
    }
    public boolean cs2_pin1Active() {
        return cs2_pin1.getState();
    }

    public boolean cs3_pin0Active() {
        return cs3_pin0.getState();
    }
    public boolean cs3_pin1Active() {
        return cs3_pin1.getState();
    }


}