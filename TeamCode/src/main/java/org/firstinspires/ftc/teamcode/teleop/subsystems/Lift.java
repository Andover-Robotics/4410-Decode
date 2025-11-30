//package org.firstinspires.ftc.teamcode.teleop.subsystems;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.arcrobotics.ftclib.hardware.motors.Motor;
//import com.arcrobotics.ftclib.hardware.motors.MotorEx;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.arcrobotics.ftclib.hardware.motors.CRServo;
//import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//@Config
//public class Lift {
//
//    private final CRServo climbLeft, climbRight;
//    private AnalogInput leftEnc, rightEnc;
//
//
//    public Lift(OpMode opMode) {
//        climbLeft = new CRServo(opMode.hardwareMap, "climbLeft");
//        climbRight = new CRServo(opMode.hardwareMap, "climbRight");
//        leftEnc = opMode.hardwareMap.get(AnalogInput.class, "leftEnc");
//        rightEnc = opMode.hardwareMap.get(AnalogInput.class, "rightEnc");
//    }
//
//    public double getRightEnc() {
//         return rightEnc.getVoltage() / 3.3 * 360;
//    }
//    public double getLeftEnc() {
//        return leftEnc.getVoltage() / 3.3 * 360;
//    }
//
//}

package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Lift {


    public static double WRAP_TOLERANCE_DEG = 90.0, MAX_ANALOG_VOLT = 3.3, kP = 0.002, kI = 0.0, kD = 0.0, kF = -0.2, POSITION_TOLERANCE_DEG = 10.0, maxPower = 0.95, spoolRad = 14, liftWidth = 178;
    public static int up = -750, down = 0, balancing = -500;

    public static boolean LEFT_INVERTED = false, RIGHT_INVERTED = false, offsetLeftSide = false;

    public double leftPower, rightPower, offset;

    private final CRServo climbLeft, climbRight;
    private final AnalogInput leftEnc, rightEnc;

    private final ContinuousAngleTracker leftTracker, rightTracker;
    private final PIDController leftPID = new PIDController(kP, kI, kD);
    private final PIDController rightPID = new PIDController(kP, kI, kD);

    public double leftTargetDeg, rightTargetDeg, leftPidOut, rightPidOut;     // continuous degrees
    private double leftZeroDeg = 0.0, rightZeroDeg = 0.0;

    private boolean closedLoopEnabled = false;         // toggle

    public static enum liftStates {
        LIFTING,
        BALANCING,
        IDLE
    }

    public liftStates liftState = liftStates.IDLE;

    public Lift(OpMode opMode) {
        climbLeft  = new CRServo(opMode.hardwareMap, "climbLeft");
        climbRight = new CRServo(opMode.hardwareMap, "climbRight");
        leftEnc    = opMode.hardwareMap.get(AnalogInput.class, "leftEnc");
        rightEnc   = opMode.hardwareMap.get(AnalogInput.class, "rightEnc");

        leftTracker  = new ContinuousAngleTracker(readAbsDeg(leftEnc), WRAP_TOLERANCE_DEG);
        rightTracker = new ContinuousAngleTracker(readAbsDeg(rightEnc), WRAP_TOLERANCE_DEG);

        leftPID.setTolerance(POSITION_TOLERANCE_DEG);
        rightPID.setTolerance(POSITION_TOLERANCE_DEG);
        zeroBoth();
    }

    public void liftUp() {
        setBothTargetDeg(up);
    }

    public void lower() {
        setBothTargetDeg(-up);
    }

//    public void balance() {
//        offset =-360 * liftWidth * Math.toRadians(Turret.orientation.getRoll(AngleUnit.DEGREES)) / (2 * Math.PI * spoolRad);
//        if (offset > 0) {
//            if (offsetLeftSide) {
//                setLeftTargetDeg(leftTargetDeg + offset);
//            } else {
//                setRightTargetDeg(rightTargetDeg + offset);
//            }
//        } else {
//            if (offsetLeftSide) {
//                setRightTargetDeg(rightTargetDeg + offset);
//            } else {
//                setLeftTargetDeg(leftTargetDeg + offset);
//            }
//        }
//    }

    /** Call this from your parent loop. Runs PIDF only when enabled. */
    public void periodic() {
        // Keep tunables synced
        leftTracker.setTolerance(WRAP_TOLERANCE_DEG);
        rightTracker.setTolerance(WRAP_TOLERANCE_DEG);
        leftPID.setPID(kP, kI, kD);
        rightPID.setPID(kP, kI, kD);
        leftPID.setTolerance(POSITION_TOLERANCE_DEG);
        rightPID.setTolerance(POSITION_TOLERANCE_DEG);

        // Update continuous encoder states
        leftTracker.update(readAbsDeg(leftEnc));
        rightTracker.update(readAbsDeg(rightEnc));

        if (!closedLoopEnabled) return; // allow driving servos manually when disabled

        // Positions in same frame as targets (continuous, zeroed)
        double leftPos  = getLeftEncContinuousDeg();
        double rightPos = getRightEncContinuousDeg();

        // PID
        leftPID.setSetPoint(leftTargetDeg);
        rightPID.setSetPoint(rightTargetDeg);
        leftPidOut  = leftPID.calculate(leftPos);
        rightPidOut = rightPID.calculate(rightPos);

        // Feedforward
        double leftF  = kF;
        double rightF = -kF;

        // Final power (clamped to [-1, 1]; optional inversion)
        leftPower  = clamp(leftPidOut + leftF, -maxPower, maxPower);
        rightPower = clamp(rightPidOut + rightF, -maxPower, maxPower);
        if (LEFT_INVERTED)  leftPower  = -leftPower;
        if (RIGHT_INVERTED) rightPower = -rightPower;

        climbLeft.set(leftPower);
        climbRight.set(rightPower);
    }

    public void enableClosedLoop(boolean enabled) { this.closedLoopEnabled = enabled; }
    public boolean isClosedLoopEnabled() { return closedLoopEnabled; }

    public void zeroBoth() {
        leftZeroDeg  = getLeftEncContinuousDegInternal();
        rightZeroDeg = getRightEncContinuousDegInternal();
    }

    public void setBothTargetDeg(double degTarget) { leftTargetDeg = -degTarget; rightTargetDeg = degTarget; }
    public void setLeftTargetDeg(double deg)  { leftTargetDeg = deg; }
    public void setRightTargetDeg(double deg) { rightTargetDeg = deg; }
    
    public double getLeftEncAbsDeg()  { return readAbsDeg(leftEnc); }
    public double getRightEncAbsDeg() { return readAbsDeg(rightEnc); }
    public double getLeftEncContinuousDeg()  { return getLeftEncContinuousDegInternal() - leftZeroDeg; }
    public double getRightEncContinuousDeg() { return getRightEncContinuousDegInternal() - rightZeroDeg; }
    public double getLeftEncContinuousRev()  { return getLeftEncContinuousDeg() / 360.0; }
    public double getRightEncContinuousRev() { return getRightEncContinuousDeg() / 360.0; }

    public boolean leftAtTarget()  { return leftPID.atSetPoint(); }
    public boolean rightAtTarget() { return rightPID.atSetPoint(); }
    
    private double getLeftEncContinuousDegInternal()  { return leftTracker.getContinuousDeg(); }
    private double getRightEncContinuousDegInternal() { return rightTracker.getContinuousDeg(); }

    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }

    private double readAbsDeg(AnalogInput ai) {
        double v = ai.getVoltage();
        double deg = (v / MAX_ANALOG_VOLT) * 360.0;
        deg %= 360.0;
        if (deg < 0) deg += 360.0;
        return deg;
    }

    /** 0–360 abs → continuous tracking with wrap detection. */
    private static class ContinuousAngleTracker {
        private double lastAbsDeg;
        private int rotationCount;
        private double toleranceDeg;

        ContinuousAngleTracker(double initialAbsDeg, double toleranceDeg) {
            this.lastAbsDeg = clamp360(initialAbsDeg);
            this.rotationCount = 0;
            this.toleranceDeg = toleranceDeg;
        }

        void setTolerance(double toleranceDeg) { this.toleranceDeg = toleranceDeg; }

        void update(double newAbsDeg) {
            newAbsDeg = clamp360(newAbsDeg);
            double delta = newAbsDeg - lastAbsDeg;
            if (delta > toleranceDeg)      rotationCount--;
            else if (delta < -toleranceDeg) rotationCount++;
            lastAbsDeg = newAbsDeg;
        }

        double getContinuousDeg() { return lastAbsDeg + 360.0 * rotationCount; }
        int getRotationCount()    { return rotationCount; }

        private static double clamp360(double deg) {
            deg %= 360.0;
            if (deg < 0) deg += 360.0;
            return deg;
        }
    }
}