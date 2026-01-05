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

    public Action shootGreen() {
        Holder h = findFirstHolderWithColor("GREEN");
        return (h == null) ? new InstantAction(() -> {}) : h.kickResetAction();
    }

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

    /* ================= OLD COLOR METHODS (KEEP SIGNATURES) ================= */

    public String getRightColor() { return rightHolder.getColor(); }
    public String getLeftColor()  { return leftHolder.getColor(); }
    public String getBackColor()  { return backHolder.getColor(); }

    /* ================= OLD SENSOR HELPERS (KEEP SIGNATURES) ================= */

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

//    public void shootMotif() {
//        List<Integer> order = planMotifOrder(motifPattern);
//
//        for (int idx : order) {
//            Holder h = holders[idx];
//
//            h.up();
//            sleepMillis((long) (kickerSleep * 1000));
//            h.down();
//
//            sleepMillis((long) (motifShootSleep * 1000));
//        }
//    }

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
        String[] colorByIdx = new String[holders.length];
        ArrayList<Integer> avail = new ArrayList<>();

        for (int i = 0; i < holders.length; i++) {
            String c = holders[i].getColor();
            colorByIdx[i] = c;
            if (!"EMPTY".equals(c)) avail.add(i);
        }

        int n = avail.size();
        if (n == 0) return Collections.emptyList();
        if (n == 1) return new ArrayList<>(avail);

        char[] pat = motif.toCharArray();
        int[] pi = buildPrefix(pat);

        List<Integer> best = null;
        int bestMotifs = -1;
        int bestProgress = -1;

        if (n == 2) {
            int a = avail.get(0), b = avail.get(1);

            int[] s1 = scorePermutation(new int[]{a, b}, colorByIdx, pat, pi);
            best = Arrays.asList(a, b);
            bestMotifs = s1[0];
            bestProgress = s1[1];

            int[] s2 = scorePermutation(new int[]{b, a}, colorByIdx, pat, pi);
            if (betterScore(s2, bestMotifs, bestProgress)) {
                best = Arrays.asList(b, a);
                bestMotifs = s2[0];
                bestProgress = s2[1];
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
            int[] sc = scorePermutation(p, colorByIdx, pat, pi);
            if (best == null || betterScore(sc, bestMotifs, bestProgress)) {
                best = Arrays.asList(p[0], p[1], p[2]);
                bestMotifs = sc[0];
                bestProgress = sc[1];
            }
        }

        return best;
    }

    private int[] scorePermutation(int[] order, String[] colorByIdx, char[] pat, int[] pi) {
        int state = 0;
        int motifs = 0;

        for (int idx : order) {
            char shot = toMotifChar(colorByIdx[idx]);

            while (state > 0 && pat[state] != shot) state = pi[state - 1];
            if (pat[state] == shot) state++;

            if (state == pat.length) {
                motifs++;
                state = pi[pat.length - 1];
            }
        }

        return new int[]{motifs, state};
    }

    private boolean betterScore(int[] sc, int bestMotifs, int bestProgress) {
        if (sc[0] != bestMotifs) return sc[0] > bestMotifs;
        return sc[1] > bestProgress;
    }

    private int[] buildPrefix(char[] p) {
        int[] pi = new int[p.length];
        int j = 0;
        for (int i = 1; i < p.length; i++) {
            while (j > 0 && p[i] != p[j]) j = pi[j - 1];
            if (p[i] == p[j]) j++;
            pi[i] = j;
        }
        return pi;
    }

    private char toMotifChar(String color) {
        if ("PURPLE".equals(color)) return 'P';
        if ("GREEN".equals(color)) return 'G';
        return 'X';
    }

    private void sleepMillis(long ms) {
        try { Thread.sleep(ms); }
        catch (InterruptedException e) { Thread.currentThread().interrupt(); }
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

        public boolean ballPresent() {
            double da = safeDistance(sensorA);
            double db = safeDistance(sensorB);
            return (da > 0 && da < DISTANCE_THRESHOLD_MM)
                    || (db > 0 && db < DISTANCE_THRESHOLD_MM);
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
            if (!ballPresent()) return "EMPTY";

            float h1 = getHue(sensorA);
            float h2 = getHue(sensorB);

            if (isGreenHue(h1) || isGreenHue(h2))   return "GREEN";
            if (isPurpleHue(h1) || isPurpleHue(h2)) return "PURPLE";

            return "UNKNOWN";
        }
    }
}
