package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Config
public class Intake {

    public static double intakePower = -1, storagePower = 0.32, reversePower = 0.8;
    public static int filterWindowSize = 7;
    public static double beamThreshold = 0.2, colorThreshold = 0.30;

    private final MotorEx motor;


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
}

