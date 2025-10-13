package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake {

    public static double intakePower = -1, storagePower = -0.3, reversePower = 0.8, gateOpen = 0.465, gateClosed = 0.55;
    private final MotorEx motor;
    public Servo gate;

    public Intake(OpMode opMode) {
        motor = new MotorEx(opMode.hardwareMap, "intake", Motor.GoBILDA.RPM_1150);
        motor.setInverted(false);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        gate = opMode.hardwareMap.servo.get("gate");
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


}
