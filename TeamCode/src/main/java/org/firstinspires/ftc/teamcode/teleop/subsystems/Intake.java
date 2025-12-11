package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake {

    public static double intakePower = -1, storagePower = -0.32, reversePower = 0.8, gateOpen = 0.1, gateClosed = 0.24;
    public static int filterWindowSize = 10;
    public static double beamThreshold = 0.2, colorThreshold = 0.30;

    private final MotorEx motor;
    public Servo gate;

    private final DigitalChannel bottomBB;
    private final DigitalChannel middleBB;
    private final DigitalChannel topBB;

    // pin0 = purple
    // pin1 = green

    public final DigitalChannel blt0;
    public final DigitalChannel blt1;

    public final DigitalChannel blm0;
    public final DigitalChannel blm1;

    public final DigitalChannel blb0;
    public final DigitalChannel blb1;

    public RevColorSensorV3 color;

    private final SlotFilter bottomSlotFilter;
    private final SlotFilter middleSlotFilter;
    private final SlotFilter topSlotFilter;

    private boolean rawBottomBeam;
    private boolean rawMiddleBeam;
    private boolean rawTopBeam;

    private boolean rawBlt0;
    private boolean rawBlt1;
    private boolean rawBlm0;
    private boolean rawBlm1;
    private boolean rawBlb0;
    private boolean rawBlb1;

    private SlotColor rawBottomColor = SlotColor.NOTHING;
    private SlotColor rawMiddleColor = SlotColor.NOTHING;
    private SlotColor rawTopColor = SlotColor.NOTHING;

    private SlotColor filteredBottom = SlotColor.NOTHING;
    private SlotColor filteredMiddle = SlotColor.NOTHING;
    private SlotColor filteredTop = SlotColor.NOTHING;

    private IntakeMode currentMode = IntakeMode.STOPPED;

    private enum IntakeMode {
        STOPPED,
        INTAKING,
        STORAGE,
        REVERSING
    }

    public Intake(OpMode opMode) {
        motor = new MotorEx(opMode.hardwareMap, "intake", Motor.GoBILDA.RPM_1150);
        motor.setInverted(false);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        gate = opMode.hardwareMap.servo.get("gate");
//        color = opMode.hardwareMap.get(RevColorSensorV3.class, "Color");

        bottomBB = opMode.hardwareMap.get(DigitalChannel.class, "bottomBB");
        middleBB = opMode.hardwareMap.get(DigitalChannel.class, "middleBB");
        topBB = opMode.hardwareMap.get(DigitalChannel.class, "topBB");

        blt0 = opMode.hardwareMap.get(DigitalChannel.class, "blt0"); //purple
        blt1 = opMode.hardwareMap.get(DigitalChannel.class, "blt1"); //green

        blm0 = opMode.hardwareMap.get(DigitalChannel.class, "blm0");
        blm1 = opMode.hardwareMap.get(DigitalChannel.class, "blm1");

        blb0 = opMode.hardwareMap.get(DigitalChannel.class, "blb0");
        blb1 = opMode.hardwareMap.get(DigitalChannel.class, "blb1");

        blt0.setMode(DigitalChannel.Mode.INPUT);
        blt1.setMode(DigitalChannel.Mode.INPUT);

        blm0.setMode(DigitalChannel.Mode.INPUT);
        blm1.setMode(DigitalChannel.Mode.INPUT);

        blb0.setMode(DigitalChannel.Mode.INPUT);
        blb1.setMode(DigitalChannel.Mode.INPUT);

        bottomSlotFilter = new SlotFilter(filterWindowSize);
        middleSlotFilter = new SlotFilter(filterWindowSize);
        topSlotFilter = new SlotFilter(filterWindowSize);

    }



    public void intake() {
        motor.set(intakePower);
        currentMode = IntakeMode.INTAKING;
    }

    public void storage() {
        motor.set(storagePower);
        currentMode = IntakeMode.STORAGE;
    }

    public void reverse() {
        motor.set(reversePower);
        resetFilters();
        currentMode = IntakeMode.REVERSING;
    }

    public void stop() {
        motor.set(0);
        currentMode = IntakeMode.STOPPED;
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

    public void periodic() {
        readRawSensors();
        updateSlotFilters();
    }

    public boolean holdingBottom() {
        return hasBall(filteredBottom);
    }

    public boolean holdingMiddle() {
        return hasBall(filteredMiddle);
    }

    public boolean holdingTop() {
        return hasBall(filteredTop);
    }

    public int storageCount() {
        return (holdingBottom()? 1 : 0) + (holdingMiddle()? 1 : 0) + (holdingTop()? 1 : 0);
    }

    public SlotColor bottomStatus() {
        return filteredBottom;
    }

    public SlotColor middleStatus() {
        return filteredMiddle;
    }

    public SlotColor topStatus() {
        return filteredTop;
    }

    public SlotColor[] getStatuses() {
        return new SlotColor[]{filteredTop, filteredMiddle, filteredBottom};
    }

    public boolean rawBottomBreakBeam() {
        return rawBottomBeam;
    }

    public boolean rawMiddleBreakBeam() {
        return rawMiddleBeam;
    }

    public boolean rawTopBreakBeam() {
        return rawTopBeam;
    }

    public SlotColor rawBottomColor() {
        return rawBottomColor;
    }

    public SlotColor rawMiddleColor() {
        return rawMiddleColor;
    }

    public SlotColor rawTopColor() {
        return rawTopColor;
    }

    public boolean rawBlt0() {
        return rawBlt0;
    }

    public boolean rawBlt1() {
        return rawBlt1;
    }

    public boolean rawBlm0() {
        return rawBlm0;
    }

    public boolean rawBlm1() {
        return rawBlm1;
    }

    public boolean rawBlb0() {
        return rawBlb0;
    }

    public boolean rawBlb1() {
        return rawBlb1;
    }

    private void readRawSensors() {
        rawBottomBeam = !bottomBB.getState();
        rawMiddleBeam = !middleBB.getState();
        rawTopBeam = !topBB.getState();

        rawBlt0 = blt0.getState();
        rawBlt1 = blt1.getState();
        rawBlm0 = blm0.getState();
        rawBlm1 = blm1.getState();
        rawBlb0 = blb0.getState();
        rawBlb1 = blb1.getState();

        rawTopColor = decodeColor(rawBlt0, rawBlt1);
        rawMiddleColor = decodeColor(rawBlm0, rawBlm1);
        rawBottomColor = decodeColor(rawBlb0, rawBlb1);
    }

    private void updateSlotFilters() {
        filteredBottom = bottomSlotFilter.addSample(rawBottomBeam, rawBottomColor);
        filteredMiddle = middleSlotFilter.addSample(rawMiddleBeam, rawMiddleColor);
        filteredTop = topSlotFilter.addSample(rawTopBeam, rawTopColor);
    }

    private boolean hasBall(SlotColor color) {
        return color != SlotColor.NOTHING;
    }

    private SlotColor decodeColor(boolean pin0, boolean pin1) {
        if (pin0 && !pin1) {
            return SlotColor.PURPLE;
        }
        if (pin1 && !pin0) {
            return SlotColor.GREEN;
        }
        if (pin0 && pin1) {
            // conflicting signals; preserve the last known filtered color through rolling average logic
            return SlotColor.UNKNOWN;
        }
        return SlotColor.NOTHING;
    }

    private void resetFilters() {
        bottomSlotFilter.clear();
        middleSlotFilter.clear();
        topSlotFilter.clear();

        filteredBottom = SlotColor.NOTHING;
        filteredMiddle = SlotColor.NOTHING;
        filteredTop = SlotColor.NOTHING;
    }

    public enum SlotColor {
        NOTHING,
        PURPLE,
        GREEN,
        UNKNOWN
    }

    private static class RollingAverage {
        private final int windowSize;
        private final java.util.ArrayDeque<Integer> samples = new java.util.ArrayDeque<>();
        private int sum = 0;

        RollingAverage(int windowSize) {
            this.windowSize = windowSize;
        }

        public double addSample(boolean active) {
            int value = active ? 1 : 0;
            samples.addLast(value);
            sum += value;
            if (samples.size() > windowSize) {
                sum -= samples.removeFirst();
            }
            return getAverage();
        }

        public double getAverage() {
            if (samples.isEmpty()) {
                return 0;
            }
            return (double) sum / samples.size();
        }

        public void clear() {
            samples.clear();
            sum = 0;
        }
    }

    private static class SlotFilter {
        private final RollingAverage beamAverage;
        private final RollingAverage purpleAverage;
        private final RollingAverage greenAverage;
        private SlotColor filteredColor = SlotColor.NOTHING;

        SlotFilter(int windowSize) {
            beamAverage = new RollingAverage(windowSize);
            purpleAverage = new RollingAverage(windowSize);
            greenAverage = new RollingAverage(windowSize);
        }

        public SlotColor addSample(boolean beamPresent, SlotColor colorReading) {
            double beamLevel = beamAverage.addSample(beamPresent);
            purpleAverage.addSample(colorReading == SlotColor.PURPLE);
            greenAverage.addSample(colorReading == SlotColor.GREEN);

            boolean anyBeamEvidence = beamLevel > 0;
            double purpleLevel = purpleAverage.getAverage();
            double greenLevel = greenAverage.getAverage();

            boolean hasConfidentColor = purpleLevel >= colorThreshold || greenLevel >= colorThreshold;
            boolean hasRecentColor = purpleLevel > 0 || greenLevel > 0;

            if (beamLevel >= beamThreshold) {
                if (hasConfidentColor) {
                    filteredColor = selectDominantColor(purpleLevel, greenLevel);
                    return filteredColor;
                }

                if (hasRecentColor) {
                    filteredColor = purpleLevel >= greenLevel ? SlotColor.PURPLE : SlotColor.GREEN;
                    return filteredColor;
                }

                filteredColor = SlotColor.UNKNOWN;
                return filteredColor;
            }

            if (anyBeamEvidence) {
                if (hasConfidentColor) {
                    filteredColor = selectDominantColor(purpleLevel, greenLevel);
                    return filteredColor;
                }

                if (hasRecentColor) {
                    filteredColor = purpleLevel >= greenLevel ? SlotColor.PURPLE : SlotColor.GREEN;
                    return filteredColor;
                }

                filteredColor = SlotColor.UNKNOWN;
                return filteredColor;
            }

            if (hasConfidentColor) {
                filteredColor = selectDominantColor(purpleLevel, greenLevel);
                return filteredColor;
            }

            if (hasRecentColor) {
                filteredColor = purpleLevel >= greenLevel ? SlotColor.PURPLE : SlotColor.GREEN;
                return filteredColor;
            }

            filteredColor = SlotColor.NOTHING;
            return filteredColor;
        }

        public void clear() {
            beamAverage.clear();
            purpleAverage.clear();
            greenAverage.clear();
            filteredColor = SlotColor.NOTHING;
        }

        private SlotColor selectDominantColor(double purpleLevel, double greenLevel) {
            if (purpleLevel > greenLevel) {
                return SlotColor.PURPLE;
            }
            if (greenLevel > purpleLevel) {
                return SlotColor.GREEN;
            }
            if (filteredColor == SlotColor.PURPLE || filteredColor == SlotColor.GREEN) {
                return filteredColor;
            }
            return SlotColor.PURPLE;
        }
    }

}

