package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Config
public class Turret {
    private final MotorEx motor;
    private PIDController controller;

    public static double p = 0.1, i = 0, d = 0.000;
    private double tolerance = 5, manualPower = 0, powerMin = 0.05;
    public static double degsPerTick = 360.0 / (145.1 * 144.0/14.0);
    public static int turretTarget = 0;

    public double power;

    private boolean isManual = false;

    public int limit = 360;

    public Turret(OpMode opMode) {
        motor = new MotorEx(opMode.hardwareMap, "turret", Motor.GoBILDA.RPM_1150);
        motor.setInverted(false);
        controller = new PIDController(p, i, d);
        controller.setTolerance(tolerance);
        controller.setSetPoint(0);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void runTo(int t) { //takes in ticks
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turretTarget = t;
    }

    public void runManual(double manual) {
        if (manual > powerMin || manual < -powerMin) {
            isManual = true;
            manualPower = manual;
        } else {
            manualPower = 0;
        }
    }

    public void runToAngle(double angle) {
        int t = (int) ((angle) / degsPerTick);
        runTo(t);
    }


    public void periodic() {
        controller.setPID(p, i, d);
        controller.setSetPoint(turretTarget);
        int pos = getPosition();
        if(isManual) {
            power = manualPower;
        } else {
            power = controller.calculate(pos);
        }
        motor.set(power);
    }

    public void resetEncoder() {
        motor.resetEncoder();
    }

    public int getPosition() {
        return motor.getCurrentPosition();
    }
    
    public double getPositionDegs() {
        return getPosition() * degsPerTick;
    }
    public double getPositionTicks() {
        return getPosition();
    }

    public int getTargetTicks() {
        return turretTarget;
    }

    public double getTargetDegs() {
        return turretTarget * degsPerTick;
    }

    public double getPower() {
        return power;
    }

}

