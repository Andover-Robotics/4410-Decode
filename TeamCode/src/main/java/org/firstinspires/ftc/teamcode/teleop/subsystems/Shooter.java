package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Shooter {

    // brr brrs
    private final MotorEx motor1;
    private final MotorEx motor2;
    public final Servo hood;

    // basic control objects
    private final PIDController controller;

    // PIDF coefficients (PID runs on RPM error to accel/decel; F is power-per-RPM feedforward)
    public static double p = 0.0025, i = 0.0, d = 0.0, f = 0.000185;
    public static boolean inverted = false;

    // note for interpoilation - distance >55, max angle = 44, distance <35, min angle = 32.5, add 1.4375
    // targeting and behavior
    public static double toleranceRPM = 75.0;   // speed window for "at speed"
    public static double minPower = 0.0;        // floor power to overcome friction
    public static double maxPower = 1.0;        // clamp
    public static double hoodMidAngleDeg = 42;
    public static double hoodFarAngleDeg = 44;
    public static double hoodMidPos = angleToPos(hoodMidAngleDeg);
    public static double hoodFarPos = angleToPos(hoodFarAngleDeg);
    public static double lowAngleLimit = 32.5, angleRange = 11.5, highServoLimit = 0.735, lowServoLimit = 1, servoPosPerAngle = (highServoLimit - lowServoLimit) / angleRange;

    // state estimation and data
    private double targetRPM = 0.0;
    private double filteredRPM = 0.0;
    private double power = 0.0;
    private boolean closedLoopEnabled = true;

    public Shooter(OpMode opMode) {
        motor1 = new MotorEx(opMode.hardwareMap, "shooterL", Motor.GoBILDA.BARE);
        motor1.setInverted(inverted);
        motor1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor1.setRunMode(Motor.RunMode.RawPower);
        motor2 = new MotorEx(opMode.hardwareMap, "shooterR", Motor.GoBILDA.BARE);
        motor2.setInverted(inverted);
        motor2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor2.setRunMode(Motor.RunMode.RawPower);

        hood = opMode.hardwareMap.servo.get("hood");
        hood.setPosition(1);

        controller = new PIDController(p, i, d);
    }

    public void setVelocity(double rpm) {
        targetRPM = rpm;
        closedLoopEnabled = true;
    }

    protected boolean inRange() {
        return (filteredRPM > targetRPM - toleranceRPM && filteredRPM < targetRPM + toleranceRPM);
    }

    //stop & reset
    public void stop() {
        targetRPM = 0.0;
        power = 0.0;
        controller.reset();
        closedLoopEnabled = true;
    }

    public void setPower(double power) {
        motor1.set(power);
        motor2.set(-power);
    }

    public void periodic() {
        filteredRPM = motor1.getVelocity() * 60 / 28;

        controller.setPID(p, i, d);

        if (closedLoopEnabled) {
            double ff = f * targetRPM;                                    // feedforward
            double pid = controller.calculate(filteredRPM, targetRPM);    // error on RPM
            power = ff + pid;

            // optional floor power when target is nonzero
            if (Math.abs(targetRPM) < 1e-3) {
                power = 0.0;
            } else {
                double s = Math.signum(targetRPM);
                power = s * Math.max(Math.abs(power), minPower);
            }
        }
        power = clamp(power, -maxPower, maxPower);
        setPower(power);
    }

    public void setHoodAngle(double angle) {
        hood.setPosition(angleToPos(angle));
    }

    public void setHoodAngleDeg(double angleDeg) {
        hood.setPosition(angleToPos(angleDeg));
    }

    protected void setHoodFar() { hood.setPosition(hoodFarPos); }

    protected void setHoodMid() { hood.setPosition(hoodMidPos); }

    // telemetry
    public double getTargetRPM() { return targetRPM; }
    public double getFilteredRPM() { return filteredRPM; }
    public double getPower() { return power; }
    public boolean atSpeed() { return Math.abs(targetRPM - filteredRPM) <= toleranceRPM; }

    //utils
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double angleToPos(double angle) {
//        return highServoLimit + ((lowServoLimit-highServoLimit) * ((angle - lowAngleLimit) / angleRange));
        return servoPosPerAngle * (angle - lowAngleLimit) + lowServoLimit;
    }
}
