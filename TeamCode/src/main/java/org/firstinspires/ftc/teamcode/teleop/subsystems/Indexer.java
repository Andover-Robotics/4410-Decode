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

import org.firstinspires.ftc.teamcode.auto.tuning.ActionHelper;
import org.firstinspires.ftc.teamcode.util.SRSHub;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.SRSHub;

import java.util.*;
import java.util.function.Supplier;

@Config
public class Indexer {

    /* ================= CONFIG ================= */


    public static double kickerLeftDown  = 0.697;
    public static double kickerLeftUp    = 0.39;
    public static double kickerRightDown = 0.627;
    public static double kickerRightUp   = 0.325;
    public static double kickerBackDown  = 0.59;
    public static double kickerBackUp    = 0.29;

    public static double kickerSleep = 0.198;

    // Rapid fire between shots (normal)
    public static double rapidShootSleep = 0.04;
    public static double autoFarSleep = 0.15;

    // Motif between shots (slow, to register motifs)
    public static double motifShootSleep = 0.35;

    public static double proximityThreshold = 28;
    public static boolean staggerSensorUpdates = true;

    public static double jiggleKickerDelta = 0.015;
    public static double jiggleKickerSleep = 0.05;

//    public static double //srshub values
//            greenHueLow = 131,
//            greenHueHigh = 139.5,
//            purpleHueLow = 139.6,
//            purpleHueHigh = 210;
    public static double
            greenHueLow = 153,
            greenHueHigh = 185,
            purpleHueLow = 185,
            purpleHueHigh = 235;


    /* ================= HOLDERS ================= */

    public final Holder rightHolder;
    public final Holder leftHolder;
    public final Holder backHolder;

    public final Holder[] holders;
    private String autoMotifPattern;
    private boolean autoMotifInitialized = false;
    private static SRSHub srsHubLeft;
    private static SRSHub srsHubRight;
    private boolean updateLeftNext = true;
    private static boolean srsInit = false;
    static SRSHub.APDS9151 rightFront = new SRSHub.APDS9151();
    static SRSHub.APDS9151 rightBack = new SRSHub.APDS9151();
    static SRSHub.APDS9151 backBottom = new SRSHub.APDS9151();
    static SRSHub.APDS9151 leftFront = new SRSHub.APDS9151();
    static SRSHub.APDS9151 leftBack = new SRSHub.APDS9151();
    static SRSHub.APDS9151 backRight = new SRSHub.APDS9151();

    /* ================= INIT ================= */

    public Indexer(OpMode opMode) {
        srsHubLeft = opMode.hardwareMap.get(SRSHub.class, "srshubLeft");
        srsHubRight = opMode.hardwareMap.get(SRSHub.class, "srshubRight");

        if (!srsInit) {
            SRSHub.Config leftConfig = new SRSHub.Config();
            leftConfig.addI2CDevice(1, rightFront);
            leftConfig.addI2CDevice(2, rightBack);
            leftConfig.addI2CDevice(3, backBottom);

            SRSHub.Config rightConfig = new SRSHub.Config();
            rightConfig.addI2CDevice(1, leftFront);
            rightConfig.addI2CDevice(2, leftBack);
            rightConfig.addI2CDevice(3, backRight);
            srsHubLeft.init(leftConfig);
            srsHubRight.init(rightConfig);
            srsInit = true;
        }

        rightHolder = new Holder(
                opMode,
                "rightKicker",
                rightFront,
                rightBack,
                kickerRightUp, kickerRightDown,
                proximityThreshold
        );

        leftHolder = new Holder(
                opMode,
                "leftKicker",
                leftFront,
                leftBack,
                kickerLeftUp, kickerLeftDown,
                proximityThreshold
        );

        backHolder = new Holder(
                opMode,
                "backKicker",
                backRight,
                backBottom,
                kickerBackUp, kickerBackDown,
                proximityThreshold
        );

        holders = new Holder[]{ rightHolder, backHolder, leftHolder};
        autoMotifPattern = getMotifPattern();
        resetIndexer();
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

    public int countBalls() {
        int balls = 0;
        for (Holder h : holders) if (h.ballPresent()) balls++;
        return balls;
    }

    public void updateSensorCache() {
        if (!staggerSensorUpdates) {
            srsHubLeft.update();
            srsHubRight.update();
            for (Holder h : holders) {
                h.updateSensorCache();
            }
            return;
        }

        if (updateLeftNext) {
            srsHubLeft.update();
            rightHolder.updateSensorCache(true, true);
            backHolder.updateSensorCache(false, true);
        } else {
            srsHubRight.update();
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

    /**
     * Rapid-fire all present holders, using rapidShootSleep between shots.
     */
    public Action shootRapidFire() {
        List<Action> actions = new ArrayList<>();
        double sleepSeconds = Turret.getRapidShootSleep(rapidShootSleep);
        for (Holder h : holders) {
            actions.add(h.kickResetAction());
            actions.add(new SleepAction(sleepSeconds));
        }
        return new SequentialAction(actions.toArray(new Action[0]));
    }
    public Action shootFarAuto() {
        List<Action> actions = new ArrayList<>();
        double sleepSeconds = Turret.getRapidShootSleep(autoFarSleep);
        for (Holder h : holders) {
            actions.add(h.kickResetAction());
            actions.add(new SleepAction(sleepSeconds));
        }
        return new SequentialAction(actions.toArray(new Action[0]));
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

    private Action buildMotifActionSupplier(Supplier<String> motifSupplier, boolean updateAutoMotif) {
        return new Action() {
            private Action builtAction;

            @Override
            public boolean run(@NonNull TelemetryPacket t) {
                if (builtAction == null) {
                    builtAction = buildMotifAction(motifSupplier.get(), updateAutoMotif);
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

    private Action buildMotifAction(String motifPattern, boolean updateAutoMotif) {

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

        int shotsPlanned = 0;

        for (int i = 0; i < motifPattern.length(); i++) {
            char target = motifPattern.charAt(i);

            Holder h = null;

            if (target == 'P') {
                if (!purple.isEmpty()) {
                    h = holders[purple.remove(0)];
                } else if (!green.isEmpty()) {
                    h = holders[green.remove(0)];
                }
            } else if (target == 'G') {
                if (!green.isEmpty()) {
                    h = holders[green.remove(0)];
                } else if (!purple.isEmpty()) {
                    h = holders[purple.remove(0)];
                }
            } else {
                if (!purple.isEmpty()) {
                    h = holders[purple.remove(0)];
                } else if (!green.isEmpty()) {
                    h = holders[green.remove(0)];
                }
            }

            if (h != null) {
                shotsPlanned++;
            }
            actions.add(h == null ? new InstantAction(() -> {}) : h.kickResetAction());
            if (i <= 2) {
                actions.add(new SleepAction(motifShootSleep));
            }
        }

        if (updateAutoMotif) {
            autoMotifPattern = rotateMotifPattern(motifPattern, shotsPlanned);
        }

        if (updateAutoMotif) {
            autoMotifPattern = rotateMotifPattern(motifPattern, shotsPlanned);
            autoMotifInitialized = true;
        }

        return new SequentialAction(actions.toArray(new Action[0]));
    }

    private static String rotateMotifPattern(String motifPattern, int offset) {
        if (motifPattern == null || motifPattern.isEmpty()) {
            return motifPattern;
        }
        int length = motifPattern.length();
        int shift = ((offset % length) + length) % length;
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

        private void kick()  { up(); }
        private void reset() { down(); }

        public Action kickResetAction() {
            return new SequentialAction(
                    new InstantAction(this::kick),
                    new SleepAction(Indexer.kickerSleep),
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
            double target = downPos - delta;
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

//                int r = Math.max(0, sensorA.red);
//                int g = Math.max(0, sensorA.green);
//                int b = Math.max(0, sensorA.blue);
//                android.graphics.Color.RGBToHSV(r, g, b, hsv);
//                hueA = hsv[0];
                hueA = sensorA.hue();
            }

            if (updateSensorB) {
                distanceB = sensorB.distanceMm();
//                int r = Math.max(0, sensorB.red);
//                int g = Math.max(0, sensorB.green);
//                int b = Math.max(0, sensorB.blue);
//                android.graphics.Color.RGBToHSV(r, g, b, hsv);
//                hueB = hsv[0];
                hueB = sensorB.hue();
            }

            boolean presentA = distanceA > 0 && distanceA < distanceThreshold;
            boolean presentB = distanceB > 0 && distanceB < distanceThreshold;

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
