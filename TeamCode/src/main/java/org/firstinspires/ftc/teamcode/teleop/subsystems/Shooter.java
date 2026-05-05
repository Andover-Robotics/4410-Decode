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
    public static double p = 0.003, i = 0.0, d = 0.0, f = 0.000125, bangBangTolerance = 20;
    public static boolean inverted = false;

    // targeting and behavior
    public static double toleranceRPM = 75.0;   // speed window for "at speed"
    public static double minPower = 0.0;        // floor power to overcome friction
    public static double maxPower = 1.0;        // clamp
    public static double hoodMidAngleDeg = 42;
    public static double hoodFarAngleDeg = 44;
    public static double hoodMidPos = angleToPos(hoodMidAngleDeg);
    public static double hoodFarPos = angleToPos(hoodFarAngleDeg);
    public static double lowAngleLimit = 35, angleRange = 15, highServoLimit = 0.68, lowServoLimit = 1, servoPosPerAngle = (highServoLimit - lowServoLimit) / angleRange;
    private double currentHoodAngle;//debugging
    private double currentServoPos;
    private double requestedHoodPos = 1.0;
    public static boolean leftEncoder = true;
    public double ff;
    public static double bangBangVelocityThresholdInPerSec = -1;

    public static boolean voltageComp = true, angleCaching = false, fullPower = false;

    // state estimation and data
    private double targetRPM = 0.0;
    private double filteredRPM = 0.0;
    private double power = 0.0;
    private double robotVelocityInPerSec = 0.0;
    private boolean closedLoopEnabled = true;


    public Shooter(OpMode opMode, SRSHubs srsHubs) {
        motor1 = new MotorEx(opMode.hardwareMap, "shooterL", Motor.GoBILDA.BARE);
        motor1.setInverted(inverted);
        motor1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor1.setRunMode(Motor.RunMode.RawPower);
        motor2 = new MotorEx(opMode.hardwareMap, "shooterR", Motor.GoBILDA.BARE);
        motor2.setInverted(inverted);
        motor2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor2.setRunMode(Motor.RunMode.RawPower);

        hood = opMode.hardwareMap.servo.get("hood");

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
        this.power = clamp(power, -maxPower, maxPower);
        closedLoopEnabled = false;
    }

    public void setRobotVelocityInPerSec(double velocityInPerSec) {
        robotVelocityInPerSec = Math.abs(velocityInPerSec);
    }

    public void periodic() {
        if (leftEncoder) {
//            filteredRPM = srsHubs.getShooterLeftVelocityTicksPerSecond() * 60 / 28;
            filteredRPM = motor1.getCorrectedVelocity() * 60 / 28;
        } else {
//            filteredRPM = srsHubs.getShooterRightVelocityTicksPerSecond() * 60 / 28;
            filteredRPM = motor1.getCorrectedVelocity() * 60 / 28;
        }

        controller.setPID(p, i, d);

        if (fullPower && Turret.trackingDistance > 110) {
            power = 1;
        } else {
            if (closedLoopEnabled) {
                if (Math.abs(targetRPM) < 1e-3) {
                    power = 0.0;
                } else if (robotVelocityInPerSec < bangBangVelocityThresholdInPerSec) {
                    power = (filteredRPM - targetRPM) < -(bangBangTolerance) ? 1.0 : 0.0;
                    ff = 0.0;
                } else {
                    ff = f * targetRPM;                                    // feedforward
                    double pid = controller.calculate(filteredRPM, targetRPM);    // error on RPM
                    power = ff + pid;
//                    double s = Math.signum(power);
//                    power = s * Math.max(Math.abs(power), minPower) * (voltageComp? 13.5 / clamp(Bot.getBatteryVoltage(), 11, 15) : 1);
                }
            }
        }
        power = clamp(power, -maxPower, maxPower);//
        motor1.set(power);
        motor2.set(-power);
        if (Math.abs(requestedHoodPos - currentServoPos) > 0.0015 || !angleCaching) {
            hood.setPosition(requestedHoodPos);
            currentHoodAngle = posToAngle(requestedHoodPos);
            currentServoPos = requestedHoodPos;
        }
    }

    public static void setFullPower(boolean t) {
        fullPower = t;
    }

    public void setHoodAngle(double angle) {
        angle = clamp(angle, lowAngleLimit, lowAngleLimit + angleRange);
        requestedHoodPos = angleToPos(angle);
    }
    public double getHoodAngle() {
        return currentHoodAngle;
    }
    public double getServoPosition(){
        return currentServoPos;
    }

    public void setHoodAngleDeg(double angleDeg) {
        setHoodAngle(angleDeg);
    }

    protected void setHoodFar() { requestedHoodPos = hoodFarPos; }

    protected void setHoodMid() { requestedHoodPos = hoodMidPos; }

    public void switchEncoder() {
        leftEncoder = !leftEncoder;
    }

    // telemetry
    public double getTargetRPM() { return targetRPM; }
    public double getFilteredRPM() { return filteredRPM; }
    public double getControllerTargetRPM() { return controller.getSetPoint(); }
    public PIDController getController() { return controller; }
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

    private static double posToAngle(double pos) {
//        return highServoLimit + ((lowServoLimit-highServoLimit) * ((angle - lowAngleLimit) / angleRange));
        return ((pos - lowServoLimit) / servoPosPerAngle) + lowAngleLimit;
    }
}
