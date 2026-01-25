package org.firstinspires.ftc.teamcode.teleop.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.auto.tuning.ActionHelper;

import java.util.*;

@Config
public class Indexer {

    /* ================= CONFIG ================= */


    public static double kickerLeftDown  = 0.705;
    public static double kickerLeftUp    = 0.39;
    public static double kickerRightDown = 0.635;
    public static double kickerRightUp   = 0.325;
    public static double kickerBackDown  = 0.60;
    public static double kickerBackUp    = 0.29;

    public static double kickerSleep = 0.20;

    // Rapid fire between shots (normal)
    public static double rapidShootSleep = 0.04;

    // Motif between shots (slow, to register motifs)
    public static double motifShootSleep = 0.28;

    static final double DISTANCE_THRESHOLD_MM = 28.0;
    public int gain = 20;
    public static boolean staggerSensorUpdates = true;

    public static double jiggleKickerDelta = 0.025;
    public static double jiggleKickerSleep = 0.05;

    /* ================= HOLDERS ================= */

    public final Holder rightHolder;
    public final Holder leftHolder;
    public final Holder backHolder;

    public final Holder[] holders;
    private int nextSensorIndex = 0;
    private final SensorTarget[] sensorReadOrder;

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

        holders = new Holder[]{ rightHolder, backHolder, leftHolder};
        sensorReadOrder = new SensorTarget[] {
                new SensorTarget(leftHolder, true),
                new SensorTarget(rightHolder, true),
                new SensorTarget(backHolder, true),
                new SensorTarget(leftHolder, false),
                new SensorTarget(rightHolder, false),
                new SensorTarget(backHolder, false)
        };
        resetIndexer();
    }

    /* ================= SENSOR GETTERS (TELEMETRY) ================= */

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
        if (!staggerSensorUpdates) {
            for (Holder h : holders) {
                h.updateSensorCache();
            }
            return;
        }

        SensorTarget target = sensorReadOrder[nextSensorIndex];
        target.holder.updateSensorCache(target.isSensorA);
        nextSensorIndex = (nextSensorIndex + 1) % sensorReadOrder.length;
    }

    /* ================= ACTIONS ================= */

    public Action shootRight() { return rightHolder.kickResetAction(); }
    public Action shootLeft()  { return leftHolder.kickResetAction(); }
    public Action shootBack()  { return backHolder.kickResetAction(); }

    public Action jiggleKickers() {
        List<Action> actions = new ArrayList<>();
        for (Holder h : holders) {
            actions.add(h.jiggleResetAction(jiggleKickerDelta, jiggleKickerSleep));
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



    public Action shootRapidFireSensor() {
        List<Action> actions = new ArrayList<>();
        for (Holder h : holders) {
            if (h.ballPresent()) {
                actions.add(h.kickResetAction());
                actions.add(new SleepAction(rapidShootSleep));
            }
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

    /* ================= MOTIF SHOOT (SLOW SLEEP) ================= */

    public Action shootMotif() {
        String motifPattern = getMotifPattern();

        List<Integer> purple = new ArrayList<>();
        List<Integer> green  = new ArrayList<>();

        for (int i = 0; i < holders.length; i++) {
            String color = holders[i].getColor();
            if ("PURPLE".equals(color)) purple.add(i);
            else if ("GREEN".equals(color)) green.add(i);
        }

        List<Action> actions = new ArrayList<>();

        for (int i = 0; i < motifPattern.length(); i++) {
            char target = motifPattern.charAt(i);

            Holder h = null;

            if (target == 'P') {
                if (!purple.isEmpty())      h = holders[purple.remove(0)];
                else if (!green.isEmpty())  h = holders[green.remove(0)];   // substitute
            } else if (target == 'G') {
                if (!green.isEmpty())       h = holders[green.remove(0)];
                else if (!purple.isEmpty()) h = holders[purple.remove(0)];  // substitute
            } else {
                // Unknown char: just shoot anything available
                if (!purple.isEmpty())      h = holders[purple.remove(0)];
                else if (!green.isEmpty())  h = holders[green.remove(0)];
            }

            actions.add(h == null ? new InstantAction(() -> {}) : h.kickResetAction());
            actions.add(new SleepAction(motifShootSleep));
        }

        return new SequentialAction(actions.toArray(new Action[0]));
    }
//
//    public class AutoShootMotifAction implements Action {
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            String motifPattern = getMotifPattern();
//
//            List<Integer> purple = new ArrayList<>();
//            List<Integer> green = new ArrayList<>();
//
//            for (int i = 0; i < holders.length; i++) {
//                String color = holders[i].getColor();
//                if ("PURPLE".equals(color)) purple.add(i);
//                else if ("GREEN".equals(color)) green.add(i);
//            }
//
//            for (int i = 0; i < motifPattern.length(); i++) {
//                char target = motifPattern.charAt(i);
//
//                Holder h = null;
//
//                if (target == 'P') {
//                    if (!purple.isEmpty()) h = holders[purple.remove(0)];
//                    else if (!green.isEmpty()) h = holders[green.remove(0)];   // substitute
//                } else if (target == 'G') {
//                    if (!green.isEmpty()) h = holders[green.remove(0)];
//                    else if (!purple.isEmpty()) h = holders[purple.remove(0)];  // substitute
//                } else {
//                    // Unknown char: just shoot anything available
//                    if (!purple.isEmpty()) h = holders[purple.remove(0)];
//                    else if (!green.isEmpty()) h = holders[green.remove(0)];
//                }
//
//                if (h != null) {
//                    try {
//                        h.kickReset();
//                        wait((long) motifShootSleep);
//                    } catch (InterruptedException e) {
//                        throw new RuntimeException(e);
//                    }
//                }
//            }
//            return false;
//        }
//    }
//
//    public AutoShootMotifAction autoShootMotifAction() {
//        return new AutoShootMotifAction();
//    }

    public static String getMotifPattern() {
        if (Bot.motif == null) {
            return "PPP"; //DEFAULT
        }

        switch (Bot.motif) {
            case GPP:
                return "GPP";
            case PGP:
                return "PGP";
            case PPG:
                return "PPG";
            case UNKNOWN:
            default:
                return "PPP";
        }
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
        private double distanceA = -1;
        private double distanceB = -1;
        private float hueA = Float.NaN;
        private float hueB = Float.NaN;
        private boolean cachedBallPresent = false;
        private String cachedColor = "EMPTY";

        public Holder(
                OpMode opMode,
                String kickerServoName,
                String leftSensorName,
                String rightSensorName,
                double upPos,
                double downPos,
                int gain
        ) {
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

//        public void kickReset() throws InterruptedException {
//            kick();
//            wait((long) kickerSleep);
//            reset();
//        }

        public Action jiggleResetAction(double delta, double sleepSeconds) {
            return new SequentialAction(
                    new InstantAction(() -> jiggle(delta)),
                    new SleepAction(sleepSeconds),
                    new InstantAction(this::reset)
            );
        }

        private void jiggle(double delta) {
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
            updateSensorCache(true);
            updateSensorCache(false);
        }

        public void updateSensorCache(boolean updateSensorA) {
            if (updateSensorA) {
                distanceA = safeDistance(sensorA);
                hueA = hueFromSensor(sensorA);
            } else {
                distanceB = safeDistance(sensorB);
                hueB = hueFromSensor(sensorB);
            }

            boolean presentA = distanceA > 0 && distanceA < DISTANCE_THRESHOLD_MM;
            boolean presentB = distanceB > 0 && distanceB < DISTANCE_THRESHOLD_MM;

            cachedBallPresent = presentA || presentB;

            if (!cachedBallPresent) {
                cachedColor = "EMPTY";
                return;
            }

            if ((presentA && isGreenHue(hueA)) || (presentB && isGreenHue(hueB))) {
                cachedColor = "GREEN";
            } else if ((presentA && isPurpleHue(hueA)) || (presentB && isPurpleHue(hueB))) {
                cachedColor = "PURPLE";
            } else {
                cachedColor = "UNKNOWN";
            }
        }

        public boolean ballPresent() {
            return cachedBallPresent;
        }

        private float getHue(RevColorSensorV3 sensor) {
            android.graphics.Color.RGBToHSV(
                    sensor.red(), sensor.green(), sensor.blue(), hsv
            );
            return hsv[0];
        }

        private boolean isGreenHue(float h)  { return h > 153 && h < 185; }
        private boolean isPurpleHue(float h) { return h > 185 && h < 235; }

        public String getColor() {
            return cachedColor;
        }

        public float hueFromSensor(RevColorSensorV3 sensor) {
            com.qualcomm.robotcore.hardware.NormalizedRGBA colors = sensor.getNormalizedColors();
            int r = Math.round(colors.red * 255f);
            int g = Math.round(colors.green * 255f);
            int b = Math.round(colors.blue * 255f);
            android.graphics.Color.RGBToHSV(r, g, b, hsv);
            return hsv[0];
        }

        public String hsvFromSensor(RevColorSensorV3 sensor) {
            com.qualcomm.robotcore.hardware.NormalizedRGBA colors = sensor.getNormalizedColors();
            int r = Math.round(colors.red * 255f);
            int g = Math.round(colors.green * 255f);
            int b = Math.round(colors.blue * 255f);
            android.graphics.Color.RGBToHSV(r, g, b, hsv);
            return Arrays.toString(hsv);
        }
    }

    private static final class SensorTarget {
        private final Holder holder;
        private final boolean isSensorA;

        private SensorTarget(Holder holder, boolean isSensorA) {
            this.holder = holder;
            this.isSensorA = isSensorA;
        }
    }
}
