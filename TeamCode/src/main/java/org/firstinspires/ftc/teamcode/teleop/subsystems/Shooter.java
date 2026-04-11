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
    public static double p = 0.0021, i = 0.0, d = 0.0, f = 0.000175;
    public static boolean inverted = false;

    // note for interpolation - distance >55, max angle = 44, distance <35, min angle = 32.5, add 1.4375
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

    public static boolean voltageComp = true;

    // state estimation and data
    private double targetRPM = 0.0;
    private double filteredRPM = 0.0;
    private double power = 0.0;
    private boolean closedLoopEnabled = true;
    private static final double TICKS_PER_REV = 28.0;

    // manual velocity estimation state (ticks + wall clock)
    private int lastLeftTicks = 0;
    private int lastRightTicks = 0;
    private long lastLeftTimeNanos = 0L;
    private long lastRightTimeNanos = 0L;


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

    public void periodic() {
        double leftRpm = calculateShooterRPM(motor1, true);
        double rightRpm = calculateShooterRPM(motor2, false);
        filteredRPM = leftEncoder ? leftRpm : rightRpm;

        controller.setPID(p, i, d);

        if (closedLoopEnabled) {
            ff = f * targetRPM;                                    // feedforward
            double pid = controller.calculate(filteredRPM, targetRPM);    // error on RPM
            power = ff + pid;

            // optional floor power when target is nonzero
            if (Math.abs(targetRPM) < 1e-3) {
                power = 0.0;
            } else {
                double s = Math.signum(power);
                power = s * Math.max(Math.abs(power), minPower) * (voltageComp? 13.5 / clamp(Bot.getBatteryVoltage(), 11, 15) : 1);
            }
        }
        power = clamp(power, -maxPower, maxPower);
        motor1.set(power);
        motor2.set(-power);
        hood.setPosition(requestedHoodPos);
    }

    public void setHoodAngle(double angle) {
        angle = clamp(angle, lowAngleLimit, lowAngleLimit + angleRange);
        currentHoodAngle = angle;   // stores the angle for telemetry
        currentServoPos = angleToPos(angle);
        requestedHoodPos = currentServoPos;
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

    /**
     * Manual shooter RPM estimate from encoder position deltas.
     * Uses seconds between samples and converts ticks/sec to RPM.
     */
    private double calculateShooterRPM(MotorEx motor, boolean leftSide) {
        long now = System.nanoTime();
        int ticks = motor.getCurrentPosition();

        long previousTime = leftSide ? lastLeftTimeNanos : lastRightTimeNanos;
        int previousTicks = leftSide ? lastLeftTicks : lastRightTicks;

        double rpm = 0.0;
        if (previousTime != 0L) {
            double dtSec = (now - previousTime) / 1_000_000_000.0;
            if (dtSec > 1e-6) {
                int deltaTicks = ticks - previousTicks;
                double ticksPerSecond = deltaTicks / dtSec;
                rpm = ticksPerSecond * 60.0 / TICKS_PER_REV;
                if (!leftSide) rpm *= -1.0;
            }
        }

        if (leftSide) {
            lastLeftTicks = ticks;
            lastLeftTimeNanos = now;
        } else {
            lastRightTicks = ticks;
            lastRightTimeNanos = now;
        }
        return rpm;
    }
}
