package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Shooter {

    // brr brrs
    private final MotorEx motor1;
    private final MotorEx motor2;

    // basic control objects
    private final PIDController controller;
    private final ElapsedTime timer = new ElapsedTime();

    // PIDF coefficients (PID runs on RPM error to accel/decel; F is power-per-RPM feedforward)
    public static double p = 0.0011, i = 0.0, d = 0.0, f = 0.000197;

    // mech conversion: motor ticks per rev and external gear ratio to flywheel
    // effective ticks per flywheel revolution = motorTicksPerRev * gearRatio
    public static double motorTicksPerRev = 28.0;
    public static double gearRatio = 1.0;
    public static boolean inverted = false;

    // targeting and behavior
    public static double toleranceRPM = 40.0;   // speed window for "at speed"
    public static double minPower = 0.0;        // floor power to overcome friction
    public static double maxPower = 1.0;        // clamp
    public static double filterAlpha = 1.0;     // 1.0 no filter, 0.1 strong smoothing

    // state estimation and data
    private double targetRPM = 0.0;
    private double measuredRPM = 0.0;
    private double filteredRPM = 0.0;
    private double power = 0.0;
    private double lastPos = 0.0;
    private double lastTime = 0.0;
    private boolean closedLoopEnabled = true;
    private boolean firstLoop = true;


    public Shooter(OpMode opMode) {
        motor1 = new MotorEx(opMode.hardwareMap, "shooterL", Motor.GoBILDA.BARE);
        motor1.setInverted(inverted);
        motor1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor1.setRunMode(Motor.RunMode.RawPower);
        motor2 = new MotorEx(opMode.hardwareMap, "shooterR", Motor.GoBILDA.BARE);
        motor2.setInverted(inverted);
        motor2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor2.setRunMode(Motor.RunMode.RawPower);


        controller = new PIDController(p, i, d);

        timer.reset();
        lastPos = motor1.getCurrentPosition();
        lastTime = timer.seconds();
    }

    public void setVelocity(double rpm) {
        targetRPM = rpm;
        closedLoopEnabled = true;
    }

    //open loop - disables pidf
    public void setManualPower(double rpm) {
        closedLoopEnabled = false;
        power = clamp(rpm / 6000, -maxPower, maxPower);
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
        // Update measurement
        double now = timer.seconds();
        double dt = now - lastTime;
        if (dt <= 0) dt = 1e-3;

        double pos = motor1.getCurrentPosition();
        double ticksPerRev = motorTicksPerRev * gearRatio;
        double ticksPerSec = (pos - lastPos) / dt;
        measuredRPM = (ticksPerSec / ticksPerRev) * 60.0;

        // low-pass filter on RPM
//        if (firstLoop) {
//            filteredRPM = measuredRPM;
//            firstLoop = false;
//        } else {
//            double a = clamp(filterAlpha, 0.0, 1.0);
//            filteredRPM = filteredRPM + a * (measuredRPM - filteredRPM);
//        }
        filteredRPM = motor1.getVelocity() * 60 / 28;

        lastPos = pos;
        lastTime = now;

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

    // telemetry
    public double getTargetRPM() { return targetRPM; }
    public double getMeasuredRPM() { return measuredRPM; }
    public double getFilteredRPM() { return filteredRPM; }
    public double getPower() { return power; }
    public boolean atSpeed() { return Math.abs(targetRPM - filteredRPM) <= toleranceRPM; }

    //utils
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
