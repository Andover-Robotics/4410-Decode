package org.firstinspires.ftc.teamcode.teleop.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.SRSHub;

import java.util.*;
import java.util.function.Supplier;

@Config
public class Indexer {

    /* ================= CONFIG ================= */
    public static double kickerLeftDown  = 0.214;
    public static double kickerLeftUp    = 0.65;
    public static double kickerRightDown = 0.22;
    public static double kickerRightUp   = 0.65;
    public static double kickerBackDown  = 0.210;
    public static double kickerBackUp    = 0.66;

    public static double kickerRightTooHigh = 0.7;

    public static double kickerSleep = 0.135;

    // Rapid fire between shots (normal)
    public static double rapidShootSleep = 0.035;

    // Motif between shots (slow, to register motifs)
    public static double motifShootSleep = 0.40;
    public static double notRapidSleep = 0.15;

    public static double proximityThreshold = 26;
    public static boolean staggerSensorUpdates = true;

    public static double jiggleKickerDelta = 0.015;
    public static double jiggleKickerSleep = 0.05;

    public static boolean shooting = false;

    public static double
            greenHueLow = 147,
            greenHueHigh = 178,
            purpleHueLow = 178,
            purpleHueHigh = 235;

    /* ================= HOLDERS ================= */

    public final Holder rightHolder;
    public final Holder leftHolder;
    public final Holder backHolder;

    public final Holder[] holders;
    private String autoMotifPattern;
    private boolean autoMotifInitialized = false;
    private boolean updateLeftNext = true;

    /* ================= INIT ================= */

    public Indexer(OpMode opMode, SRSHubs srsHubs) {
        rightHolder = new Holder(
                opMode,
                "rightKicker",
                srsHubs.rightFront,
                srsHubs.rightBack,
                kickerRightUp, kickerRightDown,
                proximityThreshold
        );

        leftHolder = new Holder(
                opMode,
                "leftKicker",
                srsHubs.leftFront,
                srsHubs.leftBack,
                kickerLeftUp, kickerLeftDown,
                proximityThreshold
        );

        backHolder = new Holder(
                opMode,
                "backKicker",
                srsHubs.backRight,
                srsHubs.backBottom,
                kickerBackUp, kickerBackDown,
                proximityThreshold
        );

        holders = new Holder[]{ rightHolder, backHolder, leftHolder};
        autoMotifPattern = getMotifPattern();
    }

    /* ================= SENSOR GETTERS (TELEMETRY) ================= */

    public RevColorSensorV3 colorRR() { return null; }
    public RevColorSensorV3 colorRL() { return null; }
    public RevColorSensorV3 colorLL() { return null; }
    public RevColorSensorV3 colorLR() { return null; }
    public RevColorSensorV3 colorBR() { return null; }
    public RevColorSensorV3 colorBL() { return null; }

    /* ================= INDEXER-LEVEL ================= */

    public void resetIndexer() {
        for (Holder h : holders) h.down();
    }

    public void rightTooHigh() {
        rightHolder.tooHigh();
    }

    public void rightReset() {
        rightHolder.reset();
    }

    public void resetDisabledSensors() {
        for (Holder h : holders) {
            h.resetDisabledSensors();
        }
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

        if (updateLeftNext) {
            rightHolder.updateSensorCache(true, true);
            backHolder.updateSensorCache(false, true);
        } else {
            leftHolder.updateSensorCache(true, true);
            backHolder.updateSensorCache(true, false);
        }
        updateLeftNext = !updateLeftNext;
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

    public Action resetKickersAction() {
        List<Action> actions = new ArrayList<>();
        for (Holder h : holders) {
            actions.add(h.resetFastAction());
        }
        return new SequentialAction(actions.toArray(new Action[0]));
    }

    /**
     * Rapid-fire all present holders, using rapidShootSleep between shots.
     */
    public Action shootRapidFire() {
        List<Action> actions = new ArrayList<>();
        double sleepSeconds = Turret.getRapidShootSleep(rapidShootSleep);
        shooting = true;

        if (!Turret.deadzone) {
            int[] rapidFireOrder = buildRapidFireMotifOrder(getMotifPattern());
            for (int i = 0; i < 3; i++) {
                Holder holder = holders[rapidFireOrder[i]];
                if (i != 2) {
                    actions.add(holder.kickResetAction());
                    actions.add(new SleepAction(sleepSeconds));
                } else {
                    actions.add(holder.longKickResetAction());
                }
            }
        } else {
            actions.add(new SleepAction(0.01));
        }

        shooting = false;
        return new SequentialAction(actions.toArray(new Action[0]));
    }

    public Action shootNotRapidFire() {
        List<Action> actions = new ArrayList<>();
        shooting = true;

        if (!Turret.deadzone) {
            int[] rapidFireOrder = buildRapidFireMotifOrder(getMotifPattern());
            for (int i = 0; i < 3; i++) {
                Holder holder = holders[rapidFireOrder[i]];
                if (i != 2) {
                    actions.add(holder.kickResetAction());
                    actions.add(new SleepAction(notRapidSleep));
                } else {
                    actions.add(holder.longKickResetAction());
                }
            }
        } else {
            actions.add(new SleepAction(0.01));
        }

        shooting = false;
        return new SequentialAction(actions.toArray(new Action[0]));
    }


    /**
     * Rapid-fire all present holders, using rapidShootSleep between shots.
     */
    public Action shootRapidFireAutoFar() {
        List<Action> actions = new ArrayList<>();

        Bot bot = Bot.getInstance();
        shooting = true;

        if (!Turret.deadzone) {
            int[] rapidFireOrder = buildRapidFireMotifOrder(getMotifPattern());
            for (int i = 0; i < 3; i++) {
                Holder holder = holders[rapidFireOrder[i]];
                if (i != 2) {
                    actions.add(holder.kickResetAction());
                } else {
                    actions.add(holder.longKickResetAction());
                    actions.add(new InstantAction(() -> Shooter.setFullPower(false)));
                }
            }
        } else {
            actions.add(new SleepAction(0.01));
        }

        shooting = false;
        return new SequentialAction(actions.toArray(new Action[0]));
    }

    private int[] buildRapidFireMotifOrder(String motifPattern) {
        List<Integer> unmatchedBallIndices = new ArrayList<>();
        List<Integer> purpleBallIndices = new ArrayList<>();
        List<Integer> greenBallIndices = new ArrayList<>();
        boolean[] used = new boolean[holders.length];

        for (int i = 0; i < holders.length; i++) {
            if (!holders[i].ballPresent()) continue;

            String color = holders[i].getColor();
            if ("PURPLE".equals(color)) {
                purpleBallIndices.add(i);
            } else if ("GREEN".equals(color)) {
                greenBallIndices.add(i);
            } else {
                unmatchedBallIndices.add(i);
            }
        }

        List<Integer> order = new ArrayList<>(holders.length);
        int targetLength = Math.min(motifPattern.length(), holders.length);

        for (int i = 0; i < targetLength; i++) {
            char target = motifPattern.charAt(i);
            Integer picked = null;

            if (target == 'P' && !purpleBallIndices.isEmpty()) {
                picked = purpleBallIndices.remove(0);
            } else if (target == 'G' && !greenBallIndices.isEmpty()) {
                picked = greenBallIndices.remove(0);
            }

            if (picked == null && !purpleBallIndices.isEmpty()) {
                picked = purpleBallIndices.remove(0);
            }
            if (picked == null && !greenBallIndices.isEmpty()) {
                picked = greenBallIndices.remove(0);
            }
            if (picked == null && !unmatchedBallIndices.isEmpty()) {
                picked = unmatchedBallIndices.remove(0);
            }

            if (picked != null && !used[picked]) {
                used[picked] = true;
                order.add(picked);
            }
        }

        for (int i = 0; i < holders.length; i++) {
            if (!used[i]) {
                order.add(i);
            }
        }

        int[] rapidFireOrder = new int[holders.length];
        for (int i = 0; i < holders.length; i++) {
            rapidFireOrder[i] = order.get(i);
        }
        return rapidFireOrder;
    }

    public Action shootRapidFireSensor() {
        List<Action> actions = new ArrayList<>();
        double sleepSeconds = Turret.getRapidShootSleep(rapidShootSleep);
        for (Holder h : holders) {
            if (h.ballPresent()) {
                actions.add(h.kickResetAction());
                actions.add(new SleepAction(sleepSeconds));
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
        return buildMotifActionSupplier(this::getMotifPattern, false);
    }

    public Action shootMotifAuto() {
        return buildMotifActionSupplier(this::getAutoMotifPattern, true);
    }

    /**
     * Auto motif shot variant that always kicks all three holders.
     * Pattern-matched shots happen first, then any leftover holders are kicked
     * to clear possible ramp-detection artifacts.
     */
    public Action shootMotifAutoClearArtifacts() {
        return buildMotifActionSupplier(this::getAutoMotifPattern, false, true);
    }

    private Action buildMotifActionSupplier(Supplier<String> motifSupplier, boolean updateAutoMotif) {
        return buildMotifActionSupplier(motifSupplier, updateAutoMotif, false);
    }

    private Action buildMotifActionSupplier(Supplier<String> motifSupplier, boolean updateAutoMotif, boolean kickAllSlots) {
        return new Action() {
            private Action builtAction;

            @Override
            public boolean run(@NonNull TelemetryPacket t) {
                if (builtAction == null) {
                    builtAction = buildMotifAction(motifSupplier.get(), updateAutoMotif, kickAllSlots);
                }
                return builtAction.run(t);
            }

            @Override
            public void preview(@NonNull Canvas canvas) {
                if (builtAction != null) {
                    builtAction.preview(canvas);
                }
            }
        };
    }

    private Action buildMotifAction(String motifPattern, boolean updateAutoMotif, boolean kickAllSlots) {

        List<Integer> purple = new ArrayList<>();
        List<Integer> green = new ArrayList<>();
        List<Integer> remaining = new ArrayList<>();

        for (int i = 0; i < holders.length; i++) {
            String color = holders[i].getColor();
            remaining.add(i);
            if ("PURPLE".equals(color)) {
                purple.add(i);
            } else if ("GREEN".equals(color)) {
                green.add(i);
            }
        }

        List<Action> actions = new ArrayList<>();

        int shotsPlanned = 0;

        for (int i = 0; i < motifPattern.length(); i++) {
            char target = motifPattern.charAt(i);

            Holder h = null;
            int holderIndex = -1;

            if (target == 'P') {
                if (!purple.isEmpty()) {
                    holderIndex = purple.remove(0);
                    h = holders[holderIndex];
                } else if (!green.isEmpty()) {
                    holderIndex = green.remove(0);
                    h = holders[holderIndex];
                }
            } else if (target == 'G') {
                if (!green.isEmpty()) {
                    holderIndex = green.remove(0);
                    h = holders[holderIndex];
                } else if (!purple.isEmpty()) {
                    holderIndex = purple.remove(0);
                    h = holders[holderIndex];
                }
            } else {
                if (!purple.isEmpty()) {
                    holderIndex = purple.remove(0);
                    h = holders[holderIndex];
                } else if (!green.isEmpty()) {
                    holderIndex = green.remove(0);
                    h = holders[holderIndex];
                }
            }

            if (h != null) {
                shotsPlanned++;
                remaining.remove((Integer) holderIndex);
            }
            actions.add(h == null ? new InstantAction(() -> {}) : h.kickResetAction());
            if (i < 2) {
                actions.add(new SleepAction(motifShootSleep));
            }
        }

        if (kickAllSlots) {
            for (int i = 0; i < remaining.size(); i++) {
                actions.add(holders[remaining.get(i)].kickResetAction());
                if (i < remaining.size() - 1) {
                    actions.add(new SleepAction(motifShootSleep));
                }
            }
        }

        if (updateAutoMotif) {
            autoMotifPattern = rotateMotifPattern(getAutoMotifPattern(), shotsPlanned);
            autoMotifInitialized = true;
        }
        return new SequentialAction(actions.toArray(new Action[0]));
    }

    private static String rotateMotifPattern(String motifPattern, int offset) {
        if (motifPattern == null || motifPattern.isEmpty()) {
            return motifPattern;
        }
        int length = motifPattern.length();
        int shift = offset % length;
        if (shift == 0) {
            return motifPattern;
        }
        return motifPattern.substring(shift) + motifPattern.substring(0, shift);
    }

    private String getAutoMotifPattern() {
        if (!autoMotifInitialized) {
            autoMotifPattern = getMotifPattern();
            autoMotifInitialized = true;
        }
        return autoMotifPattern;
    }

    public void rampDetectionOffsetAutoMotifBy(int offset) {
        autoMotifPattern = rotateMotifPattern(getMotifPattern(), offset); //get pure motif for this
        autoMotifInitialized = true;
    }


    public String getMotifPattern() {
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
        public final SRSHub.APDS9151 sensorA;
        final SRSHub.APDS9151 sensorB;

        private final double upPos;
        private final double downPos;
        private final double distanceThreshold;

        private final float[] hsv = new float[3];
        private double distanceA = -1;
        private double distanceB = -1;
        private float hueA = Float.NaN;
        private float hueB = Float.NaN;
        private boolean cachedBallPresent = false;
        private String cachedColor = "EMPTY";
        private int consecutiveCoveredAfterKickA = 0;
        private int consecutiveCoveredAfterKickB = 0;
        private long coveredStreakStartNsA = -1;
        private long coveredStreakStartNsB = -1;
        private boolean sensorADisabled = false;
        private boolean sensorBDisabled = false;
        private static final int SENSOR_DISABLE_KICK_COUNT = 3;
        private static final long SENSOR_DISABLE_MIN_STREAK_NS = 5_000_000_000L;

        public Holder(
                OpMode opMode,
                String kickerServoName,
                SRSHub.APDS9151 sensorA,
                SRSHub.APDS9151 sensorB,
                double upPos,
                double downPos,
                double distanceThreshold
        ) {
            kicker = opMode.hardwareMap.servo.get(kickerServoName);
            this.sensorA = sensorA;
            this.sensorB = sensorB;
            this.upPos = upPos;
            this.downPos = downPos;
            this.distanceThreshold = distanceThreshold;
        }

        public void up()   { kicker.setPosition(upPos); }
        public void down() { kicker.setPosition(downPos); }

        private void kick()  {
            updateSensorDisableStateAfterKick();
            up();
        }

        private void reset() {
            down();
        }

        private void tooHigh() {
            kicker.setPosition(Indexer.kickerRightTooHigh);
        }

        public Action resetAction() {
            return new InstantAction(this::reset);
        }

        public Action kickResetAction() {
            return new SequentialAction(
                    new InstantAction(this::kick),
                    new SleepAction(Indexer.kickerSleep),
                    new InstantAction(this::reset)
            );
        }
        public Action resetFastAction() {
            return new SequentialAction(
                    new InstantAction(this::kick),
                    new SleepAction(0.0001),
                    new InstantAction(this::reset)
            );
        }

        public Action longKickResetAction() {
            return new SequentialAction(
                    new InstantAction(this::kick),
                    new SleepAction(Indexer.kickerSleep + rapidShootSleep),
                    new InstantAction(this::reset)
            );
        }

        public Action jiggleResetAction(double delta, double sleepSeconds) {
            return new SequentialAction(
                    new InstantAction(() -> jiggle(delta)),
                    new SleepAction(sleepSeconds),
                    new InstantAction(this::reset)
            );
        }

        private void jiggle(double delta) {
            double target = downPos + delta;
            kicker.setPosition(clampPosition(target));
        }

        private double clampPosition(double position) {
            return Math.max(0.0, Math.min(1.0, position));
        }

        public void updateSensorCache() {
            updateSensorCache(true, true);
        }

        public void updateSensorCache(boolean updateSensorA) {
            updateSensorCache(updateSensorA, !updateSensorA);
        }

        public void updateSensorCache(boolean updateSensorA, boolean updateSensorB) {
            if (updateSensorA) {
                distanceA = sensorA.distanceMm();
                if (sensorADisabled) {
                    boolean stillCovered = distanceA > 0 && distanceA < distanceThreshold;
                    if (!stillCovered) {
                        sensorADisabled = false;
                        consecutiveCoveredAfterKickA = 0;
                        coveredStreakStartNsA = -1;
                    }
                }

                if (!sensorADisabled) {
//                int r = Math.max(0, sensorA.red);
//                int g = Math.max(0, sensorA.green);
//                int b = Math.max(0, sensorA.blue);
//                android.graphics.Color.RGBToHSV(r, g, b, hsv);
//                hueA = hsv[0];
                    hueA = sensorA.hue();
                }
            }

            if (updateSensorB) {
                distanceB = sensorB.distanceMm();
                if (sensorBDisabled) {
                    boolean stillCovered = distanceB > 0 && distanceB < distanceThreshold;
                    if (!stillCovered) {
                        sensorBDisabled = false;
                        consecutiveCoveredAfterKickB = 0;
                        coveredStreakStartNsB = -1;
                    }
                }

                if (!sensorBDisabled) {
//                int r = Math.max(0, sensorB.red);
//                int g = Math.max(0, sensorB.green);
//                int b = Math.max(0, sensorB.blue);
//                android.graphics.Color.RGBToHSV(r, g, b, hsv);
//                hueB = hsv[0];
                    hueB = sensorB.hue();
                }
            }

            boolean presentA = !sensorADisabled && distanceA > 0 && distanceA < distanceThreshold;
            boolean presentB = !sensorBDisabled && distanceB > 0 && distanceB < distanceThreshold;

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

        private void updateSensorDisableStateAfterKick() {
            if (!sensorADisabled) {
                boolean presentA = distanceA > 0 && distanceA < distanceThreshold;
                long nowNs = System.nanoTime();
                if (presentA) {
                    if (consecutiveCoveredAfterKickA == 0) {
                        coveredStreakStartNsA = nowNs;
                    }
                    consecutiveCoveredAfterKickA++;
                } else {
                    consecutiveCoveredAfterKickA = 0;
                    coveredStreakStartNsA = -1;
                }

                if (consecutiveCoveredAfterKickA >= SENSOR_DISABLE_KICK_COUNT
                        && coveredStreakStartNsA > 0
                        && nowNs - coveredStreakStartNsA >= SENSOR_DISABLE_MIN_STREAK_NS) {
                    sensorADisabled = true;
                }
            }

            if (!sensorBDisabled) {
                boolean presentB = distanceB > 0 && distanceB < distanceThreshold;
                long nowNs = System.nanoTime();
                if (presentB) {
                    if (consecutiveCoveredAfterKickB == 0) {
                        coveredStreakStartNsB = nowNs;
                    }
                    consecutiveCoveredAfterKickB++;
                } else {
                    consecutiveCoveredAfterKickB = 0;
                    coveredStreakStartNsB = -1;
                }

                if (consecutiveCoveredAfterKickB >= SENSOR_DISABLE_KICK_COUNT
                        && coveredStreakStartNsB > 0
                        && nowNs - coveredStreakStartNsB >= SENSOR_DISABLE_MIN_STREAK_NS) {
                    sensorBDisabled = true;
                }
            }
        }

        private boolean isGreenHue(float h)  { return h > greenHueLow && h < greenHueHigh; }
        private boolean isPurpleHue(float h) { return h > purpleHueLow && h < purpleHueHigh; }

        public String getColor() {
            return cachedColor;
        }

        public double getDistanceA() {
            return distanceA;
        }

        public double getDistanceB() {
            return distanceB;
        }

        public float getHueA() {
            return hueA;
        }

        public float getHueB() {
            return hueB;
        }

        public boolean isSensorADisabled() {
            return sensorADisabled;
        }

        public boolean isSensorBDisabled() {
            return sensorBDisabled;
        }

        public int disabledSensorCount() {
            int count = 0;
            if (sensorADisabled) count++;
            if (sensorBDisabled) count++;
            return count;
        }

        public void resetDisabledSensors() {
            sensorADisabled = false;
            sensorBDisabled = false;
            consecutiveCoveredAfterKickA = 0;
            consecutiveCoveredAfterKickB = 0;
            coveredStreakStartNsA = -1;
            coveredStreakStartNsB = -1;
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
}
