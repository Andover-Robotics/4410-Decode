package org.firstinspires.ftc.teamcode.teleop.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
public class Turret {
    private final MotorEx motor;
    private PIDController controller;
    private final ElapsedTime timer = new ElapsedTime();

    private IMU imu;

    private Limelight3A limelight;
    public LLResult llResult;
    public boolean autoAimEnabled = false, imuFollow = false;

    public static double p = 0.0105, i = 0, d = 0.00065, p2 = 0.008, i2 = 0, d2 = 0.0003, manualPower = 0, dA = 1, wraparoundTime = 0.35, timerTolerance = 0.15;
    private double tolerance = 5, powerMin = 0.05, degsPerTick = 360.0 / (145.1 * 104.0/10.0), ticksPerRev = 360 / degsPerTick;

    public double ty, tarea, td, power, lastTime;

    private boolean isManual = false, wraparound = false;

    public double setPoint = 0, pos = 0, highLimit = 185, lowLimit = -185, highLimitTicks = highLimit / degsPerTick, lowLimitTicks = lowLimit/degsPerTick;

    public Turret(OpMode opMode) {
        motor = new MotorEx(opMode.hardwareMap, "turret", Motor.GoBILDA.RPM_1150);
        motor.setInverted(false);
        controller = new PIDController(p, i, d);
        controller.setTolerance(tolerance);
        controller.setSetPoint(0);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        imu = opMode.hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);
        imu.resetYaw();

        // initialize limelight
        limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        timer.reset();
        lastTime = timer.seconds();
    }

    public void enableAutoAim(boolean enable) {
        autoAimEnabled = enable;
    }

    public void runToAngle(double angle) {
        if (angle > highLimit) {
            angle = angle - 360;
            wraparound = true;
        } else if (angle < lowLimit) {
            angle = angle + 360;
            wraparound = true;
        } else {
            wraparound = false;
        }
        angle = Math.min(Math.max(lowLimit, angle), highLimit);
        int t = (int) ((angle) / degsPerTick);
        runTo(t);
    }

    private void runTo(int t) { //takes in ticks
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        setPoint = t;
    }

    public void runManual(double manual) {
        if (manual > powerMin || manual < -powerMin) {
            isManual = true;
            manualPower = manual;
        } else {
            manualPower = 0;
        }
    }

    public void periodic() {
        //setPoint = 0;
        //pos = 0;
        llResult = limelight.getLatestResult();
        controller.setPID(p, i, d);
        if ((llResult != null && llResult.isValid() && autoAimEnabled) && (!wraparound || (wraparound == (timer.seconds() - lastTime > wraparoundTime)))) {
            wraparound = false;
            ty = llResult.getTy();
            tarea = llResult.getTa();
            runToAngle(getPositionDegs()+ty);
            controller.setPID(p, i, d);
            lastTime = timer.seconds();
        } else if ((timer.seconds()-lastTime) > (wraparoundTime + timerTolerance) || !autoAimEnabled){
            if (imuFollow) {
                runToAngle(getHeading() + (45 * ((Bot.alliance == Bot.allianceOptions.BLUE_ALLIANCE)? -1 : 1)));
                controller.setPID(p2, i2, d2);
            }
        }
        pos = getPosition();
        controller.setSetPoint(setPoint);

        if(isManual || manualPower != 0) {
            power = manualPower;
        } else {
            power = controller.calculate(pos);
        }
        double maxPower = 1;
        power = Math.max(-maxPower, Math.min(maxPower, power));

        td = (dA / Math.sqrt(tarea)) * Math.cos(Math.toRadians(65));

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

    public double getTargetTicks() {
        return setPoint;
    }

    public double getTargetDegs() {
        return setPoint * degsPerTick;
    }

    public double getPower() {
        return power;
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public void resetHeading() {
        imu.resetYaw();
    }

}
