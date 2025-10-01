package org.firstinspires.ftc.teamcode.teleop.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Config
public class Turret {
    private final MotorEx motor;
    private PIDController controller;

    private Limelight3A limelight;
    private boolean autoAimEnabled = false;
    public static double correctionMultiplier = 0.5;

    public static double txAlpha = 0.2;

    private double filteredTx = 0;

    public static double p = 0.1, i = 0, d = 0.000, p2 = 0.035, i2 = 0, d2 = 0, manualPower = 0;
    private double tolerance = 5, powerMin = 0.05;
    public static double degsPerTick = 360.0 / (145.1 * 144.0/14.0);
    public static int turretTarget = 0;

    public static double ty;
    public double power;

    private boolean isManual = false;

    public int limit = 360;

    public double setPoint = 0, pos = 0;

    public Turret(OpMode opMode) {
        motor = new MotorEx(opMode.hardwareMap, "turret", Motor.GoBILDA.RPM_1150);
        motor.setInverted(false);
        controller = new PIDController(p, i, d);
        controller.setTolerance(tolerance);
        controller.setSetPoint(0);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // initialize limelight
        limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
    }

    public void enableAutoAim(boolean enable) {
        autoAimEnabled = enable;
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
        setPoint = 0;
        pos = 0;
        if (autoAimEnabled) {
            LLResult llResult = limelight.getLatestResult();
            if (llResult != null && llResult.isValid()) {
                ty = llResult.getTy();
                setPoint = 0;
                pos = -1 * ty;
            }
            controller.setPID(p2, i2, d2);
        } else {
            setPoint = turretTarget;
            pos = getPosition();
            controller.setPID(p, i, d);
        }
        controller.setSetPoint(setPoint);

        if(isManual || manualPower != 0) {
            power = manualPower;
        } else {
            power = controller.calculate(pos);
        }
        double maxPower = 0.85;
        power = Math.max(-maxPower, Math.min(maxPower, power));

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

