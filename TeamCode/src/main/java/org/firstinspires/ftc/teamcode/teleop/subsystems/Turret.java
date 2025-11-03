package org.firstinspires.ftc.teamcode.teleop.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
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

import java.util.ArrayList;

@Config
public class Turret {
    private final MotorEx motor;
    private PIDController controller;
    private final ElapsedTime timer = new ElapsedTime();

    private IMU imu;

    private Limelight3A limelight;
    public LLResult llResult;

    public Shooter shooter;

    public boolean aprilTracking = true, imuFollow = true, shooterActive = true;

    public static double p = 0.0115, i = 0, d = 0.0005, p2 = 0.008, i2 = 0, d2 = 0.0003, manualPower = 0, dA = 149, wraparoundTime = 0.35, timerTolerance = 0.15, distanceOffset = 3, llRearOffsetInches = 18;
    private double tolerance = 5, powerMin = 0.05, degsPerTick = 360.0 / (145.1 * 104.0/10.0), ticksPerRev = 360 / degsPerTick, shooterA = 197821.985, shooterC = 1403235.28, shooterF = -184.70009, shooterG = -6.47357, shooterH = 1499.98464, shooterI = 9403.26397;

    public double txAvg, tyAvg, power, lastTime, setPoint = 0, pos = 0, highLimit = 185, lowLimit = -185, highLimitTicks = highLimit / degsPerTick, lowLimitTicks = lowLimit/degsPerTick;

    public static double tx, ty, distance, tAngle, tOffset, shooterRpm = 0, avgCount = 8;
    public static YawPitchRollAngles orientation;

    public int startingOffset = 0;

    public ArrayList<Double> txArr, tyArr;

    private boolean isManual = false, wraparound = false;

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

        shooter = new Shooter(opMode);

        timer.reset();
        lastTime = timer.seconds();
        startingOffset = 45 * ((Bot.alliance == Bot.allianceOptions.BLUE_ALLIANCE)? -1 : 1);
        txArr = new ArrayList<>(0);
        tyArr = new ArrayList<>(0);
    }

    public void enableFullAuto(boolean on) {
        enableAutoAim(on);
        enableShooter(on);
    }
    
    public void enableAutoAim(boolean on) {
        enableAprilTracking(on);
        enableImuFollow(on);
    }

    public void enableAprilTracking(boolean enable) {
        aprilTracking = enable;
    }

    public void enableShooter(boolean enable) {
        shooterActive = enable;
    }

    public void enableImuFollow(boolean enable) {
        imuFollow = enable;
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

    public void filtertx() {
        txArr.add(tx);
        if (txArr.size() > avgCount) {
            txArr.remove(0);
        }
        double tempTxAvg = 0;
        for (double i : txArr) {
            tempTxAvg += i;
        }
        txAvg = tempTxAvg / txArr.size();
    }

    public void periodic() {
        if (Bot.alliance == Bot.allianceOptions.BLUE_ALLIANCE) {
            if (Bot.startingPos == Bot.startingPosition.FAR) {
                startingOffset = -45;
            } else {
                startingOffset = 0;
            }
        } else {
            if (Bot.startingPos == Bot.startingPosition.FAR) {
                startingOffset = 45;
            } else {
                startingOffset = 0;
            }
        }
        orientation = imu.getRobotYawPitchRollAngles();
        llResult = limelight.getLatestResult();
        pos = getPosition();
        controller.setPID(p, i, d);
        if ((llResult != null && llResult.isValid() && aprilTracking) && (!wraparound || (wraparound == (timer.seconds() - lastTime > wraparoundTime)))) { //checks if LL valid, and its not in the middle of wrapping around
            wraparound = false;
            tx = llResult.getTx();
            ty = llResult.getTy();
            filtertx();
            runToAngle(getPositionDegs()+ty);//not using avg here
//            ty = llResult.getTy();
//            setPoint = 0;
//            pos = -1 * ty;
            controller.setPID(p, i, d);
            lastTime = timer.seconds();
        } else if ((timer.seconds()-lastTime) > (wraparoundTime + timerTolerance) || !aprilTracking){
            if (imuFollow) {
                runToAngle(getHeading() + startingOffset) ;
                controller.setPID(p2, i2, d2);
            }
        }
        controller.setSetPoint(setPoint);

        if(isManual || manualPower != 0) {
            power = manualPower;
        } else {
            power = controller.calculate(pos);
        }

        double maxPower = 1;
        power = Math.max(-maxPower, Math.min(maxPower, power));

        tAngle = getPositionDegs()-getHeading() - startingOffset;
        tOffset = llRearOffsetInches * Math.cos(Math.toRadians(tAngle));
        distance = (29.5 - 17) / Math.tan(Math.toRadians(25 - txAvg)) - distanceOffset + tOffset;
        shooterRpm = shooterF * Math.sqrt(Math.abs(shooterG * distance + shooterH)) + shooterI; //Math.sqrt(shooterA * (distance) + shooterC);

        if (shooterActive) {
            shooter.periodic();
            shooter.setVelocity(shooterRpm);
        } else {
            shooter.setPower(0);
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
        orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public double getRoll() {
        orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getRoll(AngleUnit.DEGREES);
    }

    public void resetHeading() {
        imu.resetYaw();
    }

}
