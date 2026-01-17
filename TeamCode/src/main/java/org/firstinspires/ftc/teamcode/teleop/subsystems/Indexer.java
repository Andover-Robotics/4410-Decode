package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.*;

@Config
public class Indexer {

    /* ================= CONFIG ================= */

    public String motifPattern = "PPG";

    public static double kickerLeftDown  = 0.705;
    public static double kickerLeftUp    = 0.45;
    public static double kickerRightDown = 0.63;
    public static double kickerRightUp   = 0.37;
    public static double kickerBackDown  = 0.60;
    public static double kickerBackUp    = 0.34;

    public static double kickerSleep = 0.25;

    // Rapid fire between shots (normal)
    public static double rapidShootSleep = 0.03;

    // Motif between shots (slow, to register motifs)
    public static double motifShootSleep = 0.30;

    public static double clogValueThreshold = 0.53;
    public static double clogKickerDelta = 0.03;
    public static double clogKickerSleep = 0.05;

    static final double DISTANCE_THRESHOLD_MM = 28.0;
    public int gain = 20;

    /* ================= HOLDERS ================= */

    public final Holder rightHolder;
    public final Holder leftHolder;
    public final Holder backHolder;

    public final Holder[] holders;

    /* ================= INIT ================= */

    public Indexer(OpMode opMode) {
        rightHolder = new Holder(
                opMode,
                "rightKicker",
                "colorRR", "colorRL",
                kickerRightUp, kickerRightDown,
                gain
        );

        leftHolder = new Holder(
                opMode,
                "leftKicker",
                "colorLL", "colorLR",
                kickerLeftUp, kickerLeftDown,
                gain
        );

        backHolder = new Holder(
                opMode,
                "backKicker",
                "colorBR", "colorBL",
                kickerBackUp, kickerBackDown,
                gain
        );

        holders = new Holder[]{ rightHolder, leftHolder, backHolder };

        resetIndexer();
    }

    /* ================= SENSOR GETTERS ================= */

    public RevColorSensorV3 colorRR() { return rightHolder.sensorA; }
    public RevColorSensorV3 colorRL() { return rightHolder.sensorB; }
    public RevColorSensorV3 colorLL() { return leftHolder.sensorA; }
    public RevColorSensorV3 colorLR() { return leftHolder.sensorB; }
    public RevColorSensorV3 colorBR() { return backHolder.sensorA; }
    public RevColorSensorV3 colorBL() { return backHolder.sensorB; }

    /* ================= INDEXER-LEVEL ================= */

    public void resetIndexer() {
        for (Holder h : holders) h.down();
    }

    public int countBalls() {
        int balls = 0;
        for (Holder h : holders) if (h.ballPresent()) balls++;
        return balls;
    }

    public void updateSensorCache() {
        for (Holder h : holders) h.updateSensorCache();
    }

    /* ================= ACTIONS ================= */

    public Action shootRight() { return rightHolder.kickResetAction(); }
    public Action shootLeft()  { return leftHolder.kickResetAction(); }
    public Action shootBack()  { return backHolder.kickResetAction(); }

    public boolean isClogDetected() {
        for (Holder h : holders) {
            if (h.isClogDetected()) {
                return true;
            }
        }
        return false;
    }

    public Action jiggleKickers() {
        List<Action> actions = new ArrayList<>();
        for (Holder h : holders) {
            actions.add(h.nudgeResetAction(clogKickerDelta, clogKickerSleep));
        }
        return new SequentialAction(actions.toArray(new Action[0]));
    }

    /**
     * Rapid-fire all present holders, using rapidShootSleep between shots.
     */
    public Action shootRapidFire() {
        List<Action> actions = new ArrayList<>();
        for (Holder h : holders) {
            actions.add(h.kickResetAction());
            actions.add(new SleepAction(rapidShootSleep));
        }
        return new SequentialAction(actions.toArray(new Action[0]));
    }

    /**
     * Shoots ONE green if any holder currently contains GREEN, otherwise no-op.
     */
    public Action shootGreen() {
        Holder h = findFirstHolderWithColor("GREEN");
        return (h == null) ? new InstantAction(() -> {}) : h.kickResetAction();
    }

    /**
     * Shoots ONE purple if any holder currently contains PURPLE, otherwise no-op.
     */
    public Action shootPurple() {
        Holder h = findFirstHolderWithColor("PURPLE");
        return (h == null) ? new InstantAction(() -> {}) : h.kickResetAction();
    }

    private Holder findFirstHolderWithColor(String color) {
        for (Holder h : holders) {
            if (color.equals(h.getColor())) return h;
        }
        return null;
    }

    /* ================= COLOR METHODS (KEEP SIGNATURES) ================= */

    public String getRightColor() { return rightHolder.getColor(); }
    public String getLeftColor()  { return leftHolder.getColor(); }
    public String getBackColor()  { return backHolder.getColor(); }

    /* ================= SENSOR HELPERS (KEEP SIGNATURES) ================= */

    private final float[] hsv = new float[3];

    public double safeDistance(RevColorSensorV3 s) {
        double d = s.getDistance(DistanceUnit.MM);
        return (Double.isNaN(d) || Double.isInfinite(d)) ? -1 : d;
    }

    public float getHue(RevColorSensorV3 sensor) {
        android.graphics.Color.RGBToHSV(
                sensor.red(), sensor.green(), sensor.blue(), hsv
        );
        return hsv[0];
    }

    public float getValue(RevColorSensorV3 sensor) {
        com.qualcomm.robotcore.hardware.NormalizedRGBA colors = sensor.getNormalizedColors();
        int r = Math.round(colors.red * 255f);
        int g = Math.round(colors.green * 255f);
        int b = Math.round(colors.blue * 255f);
        android.graphics.Color.RGBToHSV(r, g, b, hsv);
        return hsv[2];
    }

    /* ================= MOTIF SHOOT (SLOW SLEEP) ================= */

    public Action shootMotif() {
        List<Integer> purple = new ArrayList<>();
        List<Integer> green = new ArrayList<>();

        for (int i = 0; i < holders.length; i++) {
            String color = holders[i].getColor();
            if ("PURPLE".equals(color)) {
                purple.add(i);
            } else if ("GREEN".equals(color)) {
                green.add(i);
            }
        }

        List<Action> actions = new ArrayList<>();
        for (int i = 0; i < motifPattern.length(); i++) {
            char target = motifPattern.charAt(i);
            Holder h = null;
            if (target == 'P') {
                h = purple.isEmpty() ? null : holders[purple.remove(0)];
            } else if (target == 'G') {
                h = green.isEmpty() ? null : holders[green.remove(0)];
            }
            actions.add(h == null ? new InstantAction(() -> {}) : h.kickResetAction());
            actions.add(new SleepAction(motifShootSleep));
        }
        return new SequentialAction(actions.toArray(new Action[0]));
    }


    private char toMotifChar(String color) {
        if ("PURPLE".equals(color)) return 'P';
        if ("GREEN".equals(color)) return 'G';
        return 'X';
    }

    /* =====================================================
       ======================= HOLDER =======================
       ===================================================== */

    public static class Holder {

        final Servo kicker;
        final RevColorSensorV3 sensorA;
        final RevColorSensorV3 sensorB;

        private final double upPos;
        private final double downPos;

        private final float[] hsv = new float[3];
        private boolean cachedBallPresent = false;
        private String cachedColor = "EMPTY";
        private SensorSnapshot cachedSensorA = null;
        private SensorSnapshot cachedSensorB = null;

        public Holder(
                OpMode opMode,
                String kickerServoName,
                String leftSensorName,
                String rightSensorName,
                double upPos,
                double downPos,
                int gain
        )
        {
            kicker = opMode.hardwareMap.servo.get(kickerServoName);

            sensorA = opMode.hardwareMap.get(RevColorSensorV3.class, leftSensorName);
            sensorB = opMode.hardwareMap.get(RevColorSensorV3.class, rightSensorName);

            this.upPos = upPos;
            this.downPos = downPos;

            setFastMode(sensorA);
            setFastMode(sensorB);
            sensorA.setGain(gain);
            sensorB.setGain(gain);
        }

        private void setFastMode(RevColorSensorV3 sensor) {
            if (sensor.getDeviceClient() instanceof LynxI2cDeviceSynch) {
                ((LynxI2cDeviceSynch) sensor.getDeviceClient())
                        .setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
            }
        }

        public void up()   { kicker.setPosition(upPos); }
        public void down() { kicker.setPosition(downPos); }

        private void kick()  { up(); }
        private void reset() { down(); }

        public Action kickResetAction() {
            return new SequentialAction(
                    new InstantAction(this::kick),
                    new SleepAction(Indexer.kickerSleep),
                    new InstantAction(this::reset)
            );
        }

        public Action nudgeResetAction(double delta, double sleepSeconds) {
            return new SequentialAction(
                    new InstantAction(() -> nudge(delta)),
                    new SleepAction(sleepSeconds),
                    new InstantAction(this::reset)
            );
        }

        private void nudge(double delta) {
            double target = downPos - delta;
            kicker.setPosition(clampPosition(target));
        }

        private double clampPosition(double position) {
            return Math.max(0.0, Math.min(1.0, position));
        }

        private double safeDistance(RevColorSensorV3 s) {
            double d = s.getDistance(DistanceUnit.MM);
            return (Double.isNaN(d) || Double.isInfinite(d)) ? -1 : d;
        }

        public void updateSensorCache() {
            SensorSnapshot a = readSensor(sensorA);
            SensorSnapshot b = readSensor(sensorB);
            cachedSensorA = a;
            cachedSensorB = b;

            cachedBallPresent = (a.distance > 0 && a.distance < DISTANCE_THRESHOLD_MM)
                    || (b.distance > 0 && b.distance < DISTANCE_THRESHOLD_MM);

            if (!cachedBallPresent) {
                cachedColor = "EMPTY";
                return;
            }

            if (isGreenHue(a.hue) || isGreenHue(b.hue)) {
                cachedColor = "GREEN";
            } else if (isPurpleHue(a.hue) || isPurpleHue(b.hue)) {
                cachedColor = "PURPLE";
            } else {
                cachedColor = "UNKNOWN";
            }
        }

        public boolean ballPresent() {
            return cachedBallPresent;
        }

        public boolean isClogDetected() {
            if (hasValidColor()) {
                return false;
            }
            return isCloggedSnapshot(cachedSensorA) || isCloggedSnapshot(cachedSensorB);
        }

        private boolean isGreenHue(float h)  { return h > 160 && h < 180; }
        private boolean isPurpleHue(float h) { return h > 180 && h < 225; }

        public String getColor() {
            return cachedColor;
        }

        private boolean hasValidColor() {
            return "GREEN".equals(cachedColor) || "PURPLE".equals(cachedColor);
        }

        private SensorSnapshot readSensor(RevColorSensorV3 sensor) {
            double d = sensor.getDistance(DistanceUnit.MM);
            HsvSnapshot hsvSnapshot = hsvFromSensor(sensor);
            double distance = (Double.isNaN(d) || Double.isInfinite(d)) ? -1 : d;
            return new SensorSnapshot(distance, hsvSnapshot.hue, hsvSnapshot.value);
        }

        private float hueFromSensor(RevColorSensorV3 sensor) {
            com.qualcomm.robotcore.hardware.NormalizedRGBA colors = sensor.getNormalizedColors();
            int r = Math.round(colors.red * 255f);
            int g = Math.round(colors.green * 255f);
            int b = Math.round(colors.blue * 255f);
            android.graphics.Color.RGBToHSV(r, g, b, hsv);
            return hsv[0];
        }

        private HsvSnapshot hsvFromSensor(RevColorSensorV3 sensor) {
            com.qualcomm.robotcore.hardware.NormalizedRGBA colors = sensor.getNormalizedColors();
            int r = Math.round(colors.red * 255f);
            int g = Math.round(colors.green * 255f);
            int b = Math.round(colors.blue * 255f);
            android.graphics.Color.RGBToHSV(r, g, b, hsv);
            return new HsvSnapshot(hsv[0], hsv[2]);
        }

        public String hsvFromSensor(RevColorSensorV3 sensor) {
            com.qualcomm.robotcore.hardware.NormalizedRGBA colors = sensor.getNormalizedColors();
            int r = Math.round(colors.red * 255f);
            int g = Math.round(colors.green * 255f);
            int b = Math.round(colors.blue * 255f);
            android.graphics.Color.RGBToHSV(r, g, b, hsv);
            return Arrays.toString(hsv);
        }

        private boolean isCloggedSnapshot(SensorSnapshot snapshot) {
            if (snapshot == null) {
                return false;
            }
            return snapshot.distance > DISTANCE_THRESHOLD_MM && snapshot.value > clogValueThreshold;
        }

        private static final class HsvSnapshot {
            private final float hue;
            private final float value;

            private HsvSnapshot(float hue, float value) {
                this.hue = hue;
                this.value = value;
            }
        }

        private static final class SensorSnapshot {
            private final double distance;
            private final float hue;
            private final float value;

            private SensorSnapshot(double distance, float hue, float value) {
                this.distance = distance;
                this.hue = hue;
                this.value = value;
            }
        }
    }
}
