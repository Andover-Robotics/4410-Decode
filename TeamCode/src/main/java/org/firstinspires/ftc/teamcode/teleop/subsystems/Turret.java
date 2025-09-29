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

    public static double p = 0.1, i = 0, d = 0.000, p2 = 0.025, i2 = 0, d2 = 0;
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
        int setPoint = turretTarget;
        int pos = getPosition();

        if (autoAimEnabled) {
            LLResult llResult = limelight.getLatestResult();
            if (llResult != null && llResult.isValid()) {
                double tx = Math.round(llResult.getTx() * 100.0) / 100.0; // horizontal offset from center
                if (Math.abs(tx) > 0.5) {
                    int txTicks = (int)(tx / degsPerTick);
                    setPoint = pos - txTicks;
                }
            }
            controller.setPID(p2, i2, d2);
        } else {
            controller.setPID(p, i, d);
        }
        controller.setSetPoint(setPoint);

        if(isManual) {
            power = manualPower;
        } else {
            power = controller.calculate(pos);
        }
        double maxPower = 0.3;
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

