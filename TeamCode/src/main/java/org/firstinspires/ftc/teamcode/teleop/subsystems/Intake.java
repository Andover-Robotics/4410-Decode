package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Intake {

    public static double intakePower = -1, storagePower = 0.32, reversePower = 1;

    private final MotorEx motor;
    public static double currentThreshold = 4500000;//mA 3300
    public static double confidence = 0.70;
    public static boolean intakeJammed = false;

    public static int JAM_SAMPLE_COUNT = 20;
    private final boolean[] jamSamples = new boolean[JAM_SAMPLE_COUNT];
    private int jamSampleIndex = 0;
    private int jammedSampleTotal = 0;


    private IntakeMode currentMode = IntakeMode.STOPPED;

    private enum IntakeMode {
        STOPPED,
        INTAKING,
        REVERSINGSLOW,
        REVERSINGFULL
    }

    public Intake(OpMode opMode) {
        motor = new MotorEx(opMode.hardwareMap, "intake", Motor.GoBILDA.RPM_1150);
        motor.setInverted(true);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

    }



    protected void intake() {
        motor.set(intakePower);
        currentMode = IntakeMode.INTAKING;
    }

    public void storage() {
        motor.set(storagePower);
        currentMode = IntakeMode.REVERSINGSLOW;
    }

    protected void reverse() {
        motor.set(reversePower);
//        resetFilters();
        currentMode = IntakeMode.REVERSINGFULL;
    }

    protected void stop() {
        motor.set(0);
        currentMode = IntakeMode.STOPPED;
    }

    public void setPower(double power) {
        motor.set(power);
    }

    public boolean isRunning() {
        return currentMode != IntakeMode.STOPPED;
    }
    public double getCurrent(){
        return motor.motorEx.getCurrent(CurrentUnit.MILLIAMPS);
    }
    public boolean isAboveCurrentThreshold(){
        return motor.motorEx.getCurrent(CurrentUnit.MILLIAMPS) > currentThreshold;

    }

    public void periodic() {
        boolean isOverCurrent = isAboveCurrentThreshold();
        boolean previousValue = jamSamples[jamSampleIndex];

        if (previousValue) {
            jammedSampleTotal--;
        }

        jamSamples[jamSampleIndex] = !Bot.unjamming && isOverCurrent;

        if (!Bot.unjamming && isOverCurrent) {
            jammedSampleTotal++;
        }

        jamSampleIndex = (jamSampleIndex + 1) % JAM_SAMPLE_COUNT;

        double jamRatio = (double) jammedSampleTotal / JAM_SAMPLE_COUNT;
        intakeJammed = jamRatio >= confidence;
        if (Bot.unjamming) {
            intakeJammed = false;
        }
    }
}
