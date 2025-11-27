package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake {

    public static double intakePower = -1, storagePower = -0.3, reversePower = 0.8, gateOpen = 0.1, gateClosed = 0.24;

    private static long BEAM_DEBOUNCE_NS = 20_000_000L; // Debounce timing (in nanoseconds) = 20 ms
    private final MotorEx motor;
    public Servo gate;

    private final DigitalChannel bottomBB;
    private final DigitalChannel middleBB;
    private final DigitalChannel topBB;

    //pin0 = green
    //pin1 = purple

    private final DigitalChannel cs1_pin0;
    private final DigitalChannel cs1_pin1;

    private final DigitalChannel cs2_pin0;
    private final DigitalChannel cs2_pin1;

    private final DigitalChannel cs3_pin0;
    private final DigitalChannel cs3_pin1;

    // records the last stable time for break beam toggles (debouncing)
    private long lastBottomChangeNs = 0;
    private long lastMiddleChangeNs = 0;
    private long lastTopChangeNs = 0;

    // records last known debounced state
    private boolean lastBottomState = false;
    private boolean lastMiddleState = false;
    private boolean lastTopState = false;

    // intake state machine beaverworks is pretty cool
    public enum IntakeState { IDLE, INTAKING, STORAGE, REVERSE, STOPPED }
    private volatile IntakeState requestedState = IntakeState.STOPPED;
    private volatile IntakeState activeState = IntakeState.STOPPED;

    // color enum
    public enum PixelColor { NONE, PURPLE, GREEN }

    public Intake(OpMode opMode) {
        motor = new MotorEx(opMode.hardwareMap, "intake", Motor.GoBILDA.RPM_1150);
        motor.setInverted(false);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        gate = opMode.hardwareMap.servo.get("gate");

        bottomBB = opMode.hardwareMap.get(DigitalChannel.class, "bottomBB");
        middleBB = opMode.hardwareMap.get(DigitalChannel.class, "middleBB");
        topBB = opMode.hardwareMap.get(DigitalChannel.class, "topBB");

        cs1_pin0 = opMode.hardwareMap.get(DigitalChannel.class, "cs1_pin0");
        cs1_pin1 = opMode.hardwareMap.get(DigitalChannel.class, "cs1_pin1");

        cs2_pin0 = opMode.hardwareMap.get(DigitalChannel.class, "cs2_pin0");
        cs2_pin1 = opMode.hardwareMap.get(DigitalChannel.class, "cs2_pin1");

        cs3_pin0 = opMode.hardwareMap.get(DigitalChannel.class, "cs3_pin0");
        cs3_pin1 = opMode.hardwareMap.get(DigitalChannel.class, "cs3_pin1");

        // digital inputs
        cs1_pin0.setMode(DigitalChannel.Mode.INPUT);
        cs1_pin1.setMode(DigitalChannel.Mode.INPUT);

        cs2_pin0.setMode(DigitalChannel.Mode.INPUT);
        cs2_pin1.setMode(DigitalChannel.Mode.INPUT);

        cs3_pin0.setMode(DigitalChannel.Mode.INPUT);
        cs3_pin1.setMode(DigitalChannel.Mode.INPUT);

        bottomBB.setMode(DigitalChannel.Mode.INPUT);
        middleBB.setMode(DigitalChannel.Mode.INPUT);
        topBB.setMode(DigitalChannel.Mode.INPUT);

        // init debounced states
        long now = System.nanoTime();
        lastBottomState = bottomBB.getState();
        lastMiddleState = middleBB.getState();
        lastTopState = topBB.getState();
        lastBottomChangeNs = now;
        lastMiddleChangeNs = now;
        lastTopChangeNs = now;

        // default to closed gate
        closeGate();
        stop();
    }

    public void intake() {
        motor.set(intakePower);
    }

    public void storage() {
        motor.set(storagePower);
    }

    public void reverse() {
        motor.set(reversePower);
    }

    public void stop() {
        motor.set(0);
    }

    public void setPower(double power) {
        motor.set(power);
    }

    public void openGate() {
        gate.setPosition(gateOpen);
    }

    public void closeGate() {
        gate.setPosition(gateClosed);
    }

    // brake beam helpers
    // this may work or not idk
    // since hardware.getState() returns a boolean
    // just for reference, original code used !getState()

    public boolean holdingBottom() {
        return !getDebouncedBeamState(bottomBB, BeamPosition.BOTTOM);
    }

    public boolean holdingMiddle() {
        return !getDebouncedBeamState(middleBB, BeamPosition.MIDDLE);
    }

    public boolean holdingTop() {
        return !getDebouncedBeamState(topBB, BeamPosition.TOP);
    }

    public int storageCount() {
        return (holdingBottom()? 1 : 0) + (holdingMiddle()? 1 : 0) + (holdingTop()? 1 : 0);
    }

    public boolean cs1_pin0Active() {
        return cs1_pin0.getState();
    }
    public boolean cs1_pin1Active() {
        return cs1_pin1.getState();
    }

    public boolean cs2_pin0Active() {
        return cs2_pin0.getState();
    }
    public boolean cs2_pin1Active() {
        return cs2_pin1.getState();
    }

    public boolean cs3_pin0Active() {
        return cs3_pin0.getState();
    }
    public boolean cs3_pin1Active() {
        return cs3_pin1.getState();
    }

    private PixelColor decodeColor(boolean pin0, boolean pin1) {
        // pin mapping
        // pin0 = false, pin1 = true ---> PURPLE
        // pin0 = true, pin1 = false ---> GREEN
        // else ---> NONE

        if (!pin0 && pin1) {
            return PixelColor.PURPLE;
        }
        if (pin0 && !pin1) {
            return PixelColor.GREEN;
        }
        else {
            return PixelColor.NONE;
        }
    }

    public PixelColor getColor1() {
        return decodeColor(cs1_pin0Active(), cs1_pin1Active());
    }

    public PixelColor getColor2() {
        return decodeColor(cs2_pin0Active(), cs2_pin1Active());
    }

    public PixelColor getColor3() {
        return decodeColor(cs3_pin0Active(), cs3_pin1Active());
    }

    public void requestIntake() {
        requestedState = IntakeState.INTAKING;
    }

    public void requestStorage() {
        requestedState = IntakeState.STORAGE;
    }

    public void requestReverse() {
        requestedState = IntakeState.REVERSE;
    }

    public void requestStop() {
        requestedState = IntakeState.STOPPED;
    }

    public IntakeState getRequestedState() {
        return requestedState;
    }

    public IntakeState getActiveState() {
        return activeState;
    }

    public void periodic() {
        // update debounced beams first
        updateDebouncedBeams();

        // state machine
        // INTAKE ---> run intakePower until either a) storage full (3 beams) b) an intermediate beam is hit
        //             when full ---> switch to STORAGE
        // STORAGE ---> apply storagePower (holding)
        // REVERSE ---> apply reversePower (failsafe)
        // STOPPED ---> stop motor

        // got this one from bezaire
        switch (requestedState) {
            case INTAKING:
                // if storage is full, go to STORAGE and don't run full intake
                if (storageCount() >= 3) {
                    activeState = IntakeState.STORAGE;
                }
                else {
                    // run intake
                    activeState = IntakeState.INTAKING;
                    motor.set(intakePower);
                }
                break;

            case STORAGE:
                activeState = IntakeState.STORAGE;
                motor.set(storagePower);
                break;

            case REVERSE:
                activeState = IntakeState.REVERSE;
                motor.set(reversePower);
                break;

            case STOPPED:
            default:
                activeState = IntakeState.STOPPED;
                motor.set(0.0);
                break;
        }

        // failsafe if all beams are triggered but motor still intaking
        if (holdingBottom() && holdingMiddle() && holdingTop()) {
            // all three detected ---> hold in storage
            activeState = IntakeState.STORAGE;
            requestedState = IntakeState.STORAGE;
            motor.set(storagePower);
        }
    }

    // debounce helpers and beam state tracking
    private enum BeamPosition { BOTTOM, MIDDLE, TOP }

    private boolean getDebouncedBeamState(DigitalChannel beam, BeamPosition pos) {
        switch (pos) {
            case BOTTOM:
                return lastBottomState;
            case MIDDLE:
                return lastMiddleState;
            case TOP:
            default:
                return lastTopState;
        }
    }

    private void updateDebouncedBeams() {
        long now = System.nanoTime();

        boolean rawButtom = bottomBB.getState();
        if (rawButtom != lastBottomState) {
            if (now - lastBottomChangeNs > BEAM_DEBOUNCE_NS) {
                lastBottomState = rawButtom;
                lastBottomChangeNs = now;
            }
        }
        else {
            lastBottomChangeNs = now;
        }

        boolean rawMiddle = middleBB.getState();
        if (rawMiddle != lastMiddleState) {
            if (now - lastMiddleChangeNs > BEAM_DEBOUNCE_NS) {
                lastMiddleState = rawMiddle;
                lastMiddleChangeNs = now;
            }
        }
        else {
            lastMiddleChangeNs = now;
        }

        boolean rawTop = topBB.getState();
        if (rawTop != lastTopState) {
            if (now - lastTopChangeNs > BEAM_DEBOUNCE_NS) {
                lastTopState = rawTop;
                lastTopChangeNs = now;
            }
        }
        else {
            lastTopChangeNs = now;
        }
    }
}