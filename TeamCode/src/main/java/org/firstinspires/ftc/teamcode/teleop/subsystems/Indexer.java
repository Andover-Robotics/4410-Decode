package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

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
    public static double motifShootSleep = 0.70;

    // If motifs are scoring in reverse, flip this in Dashboard
    public static boolean reverseMotifOrder = true;

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

    /* ================= SENSOR GETTERS (TELEMETRY COMPAT) ================= */

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

    /* ================= MOTIF SHOOT (SLOW SLEEP) ================= */

    public Action shootMotif() {
        List<Integer> order = planMotifOrder(motifPattern);
        if (order.isEmpty()) return new InstantAction(() -> {});

        List<Action> actions = new ArrayList<>();
        for (int idx : order) {
            Holder h = holders[idx];
            actions.add(new InstantAction(h::up));
            actions.add(new SleepAction(kickerSleep));
            actions.add(new InstantAction(h::down));
            actions.add(new SleepAction(motifShootSleep));
        }
        return new SequentialAction(actions.toArray(new Action[0]));
    }


    /* ================= MOTIF ORDER (SIMPLE, NO SKIPS) ================= */

    private List<Integer> planMotifOrder(String motif) {
        String pattern = reverseMotifOrder
                ? new StringBuilder(motif).reverse().toString()
                : motif;
        char[] pat = pattern.toCharArray();
        char[] shotByIdx = new char[holders.length];
        ArrayList<Integer> avail = new ArrayList<>();

        for (int i = 0; i < holders.length; i++) {
            char shot = toMotifChar(holders[i].getColor());
            shotByIdx[i] = shot;
            if (shot != 'X') avail.add(i);
        }

        int n = avail.size();
        if (n == 0) return Collections.emptyList();
        if (n == 1) return new ArrayList<>(avail);

        List<Integer> best = null;
        int bestMatches = -1;
        int bestPrefix = -1;

        if (n == 2) {
            int a = avail.get(0), b = avail.get(1);

            int[] s1 = scorePermutation(new int[]{a, b}, shotByIdx, pat);
            best = Arrays.asList(a, b);
            bestMatches = s1[0];
            bestPrefix = s1[1];

            int[] s2 = scorePermutation(new int[]{b, a}, shotByIdx, pat);
            if (s2[0] > bestMatches || (s2[0] == bestMatches && s2[1] > bestPrefix)) {
                best = Arrays.asList(b, a);
                bestMatches = s2[0];
                bestPrefix = s2[1];
            }
            return best;
        }

        int a = avail.get(0), b = avail.get(1), c = avail.get(2);

        int[][] perms = new int[][]{
                {a, b, c},
                {a, c, b},
                {b, a, c},
                {b, c, a},
                {c, a, b},
                {c, b, a}
        };

        for (int[] p : perms) {
            int[] sc = scorePermutation(p, shotByIdx, pat);
            if (best == null || sc[0] > bestMatches || (sc[0] == bestMatches && sc[1] > bestPrefix)) {
                best = Arrays.asList(p[0], p[1], p[2]);
                bestMatches = sc[0];
                bestPrefix = sc[1];
            }
        }

        return best;
    }

    private int[] scorePermutation(int[] order, char[] shotByIdx, char[] pat) {
        int matches = 0;
        int prefix = 0;
        boolean prefixActive = true;

        int limit = Math.min(order.length, pat.length);
        for (int i = 0; i < limit; i++) {
            char shot = shotByIdx[order[i]];
            if (shot == pat[i]) {
                matches++;
                if (prefixActive) {
                    prefix++;
                }
            } else {
                prefixActive = false;
            }
        }

        return new int[]{matches, prefix};
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

            sensorA.setGain(gain);
            sensorB.setGain(gain);
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

        private double safeDistance(RevColorSensorV3 s) {
            double d = s.getDistance(DistanceUnit.MM);
            return (Double.isNaN(d) || Double.isInfinite(d)) ? -1 : d;
        }

        public void updateSensorCache() {
            double da = safeDistance(sensorA);
            double db = safeDistance(sensorB);
            cachedBallPresent = (da > 0 && da < DISTANCE_THRESHOLD_MM)
                    || (db > 0 && db < DISTANCE_THRESHOLD_MM);

            if (!cachedBallPresent) {
                cachedColor = "EMPTY";
                return;
            }

            float h1 = getHue(sensorA);
            float h2 = getHue(sensorB);

            if (isGreenHue(h1) || isGreenHue(h2)) {
                cachedColor = "GREEN";
            } else if (isPurpleHue(h1) || isPurpleHue(h2)) {
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

        private boolean isGreenHue(float h)  { return h > 160 && h < 180; }
        private boolean isPurpleHue(float h) { return h > 180 && h < 225; }

        public String getColor() {
            return cachedColor;
        }
    }
}
