package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake {

    public static double intakePower = -1, storagePower = -0.32, reversePower = 0.8, gateOpen = 0.1, gateClosed = 0.24;
    private final MotorEx motor;
    public Servo gate;

    private final DigitalChannel bottomBB;
    private final DigitalChannel middleBB;
    private final DigitalChannel topBB;

    //pin0 = green
    //pin1 = purple

    public final DigitalChannel blt0;
    public final DigitalChannel blt1;

    public final DigitalChannel blm0;
    public final DigitalChannel blm1;

    public final DigitalChannel blb0;
    public final DigitalChannel blb1;

    public RevColorSensorV3 color;

    public Intake(OpMode opMode) {
        motor = new MotorEx(opMode.hardwareMap, "intake", Motor.GoBILDA.RPM_1150);
        motor.setInverted(false);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        gate = opMode.hardwareMap.servo.get("gate");
//        color = opMode.hardwareMap.get(RevColorSensorV3.class, "Color");

        bottomBB = opMode.hardwareMap.get(DigitalChannel.class, "bottomBB");
        middleBB = opMode.hardwareMap.get(DigitalChannel.class, "middleBB");
        topBB = opMode.hardwareMap.get(DigitalChannel.class, "topBB");

        blt0 = opMode.hardwareMap.get(DigitalChannel.class, "blt0"); //purple
        blt1 = opMode.hardwareMap.get(DigitalChannel.class, "blt1"); //green

        blm0 = opMode.hardwareMap.get(DigitalChannel.class, "blm0");
        blm1 = opMode.hardwareMap.get(DigitalChannel.class, "blm1");

        blb0 = opMode.hardwareMap.get(DigitalChannel.class, "blb0");
        blb1 = opMode.hardwareMap.get(DigitalChannel.class, "blb1");

        blt0.setMode(DigitalChannel.Mode.INPUT);
        blt1.setMode(DigitalChannel.Mode.INPUT);

        blm0.setMode(DigitalChannel.Mode.INPUT);
        blm1.setMode(DigitalChannel.Mode.INPUT);

        blb0.setMode(DigitalChannel.Mode.INPUT);
        blb1.setMode(DigitalChannel.Mode.INPUT);

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

    public String blbColor() {
        if (blb0.getState()) {
            return "Purple";
        } else if (blb1.getState()) {
            return "Green";
        } else {
            return "Nothing";
        }
    }

    public String blmColor() {
        if (blm0.getState()) {
            return "Purple";
        } else if (blm1.getState()) {
            return "Green";
        } else {
            return "Nothing";
        }
    }

    public String bltColor() {
        if (blt0.getState()) {
            return "Purple";
        } else if (blt1.getState()) {
            return "Green";
        } else {
            return "Nothing";
        }
    }

}