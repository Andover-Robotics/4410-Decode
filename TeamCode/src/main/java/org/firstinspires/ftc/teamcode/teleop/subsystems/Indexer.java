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
    public static double shootSleep  = 0.03;

    static final double DISTANCE_THRESHOLD_MM = 28.0;
    public int gain = 20;

    /* ================= HARDWARE ================= */

    public Servo leftKicker, rightKicker, backKicker;

    public RevColorSensorV3 colorRR, colorRL;
    public RevColorSensorV3 colorLL, colorLR;
    public RevColorSensorV3 colorBR, colorBL;

    private final float[] hsv = new float[3];

    /* ================= INIT ================= */

    public Indexer(OpMode opMode) {
        leftKicker  = opMode.hardwareMap.servo.get("leftKicker");
        rightKicker = opMode.hardwareMap.servo.get("rightKicker");
        backKicker  = opMode.hardwareMap.servo.get("backKicker");

        colorRR = opMode.hardwareMap.get(RevColorSensorV3.class, "colorRR");
        colorRL = opMode.hardwareMap.get(RevColorSensorV3.class, "colorRL");
        colorLL = opMode.hardwareMap.get(RevColorSensorV3.class, "colorLL");
        colorLR = opMode.hardwareMap.get(RevColorSensorV3.class, "colorLR");
        colorBR = opMode.hardwareMap.get(RevColorSensorV3.class, "colorBR");
        colorBL = opMode.hardwareMap.get(RevColorSensorV3.class, "colorBL");

        colorRR.setGain(gain);
        colorRL.setGain(gain);
        colorLL.setGain(gain);
        colorLR.setGain(gain);
        colorBR.setGain(gain);
        colorBL.setGain(gain);
    }

    /* ================= KICKERS ================= */

    public void rightUp()  { rightKicker.setPosition(kickerRightUp); }
    public void rightDown(){ rightKicker.setPosition(kickerRightDown); }
    public void leftUp()   { leftKicker.setPosition(kickerLeftUp); }
    public void leftDown() { leftKicker.setPosition(kickerLeftDown); }
    public void backUp()   { backKicker.setPosition(kickerBackUp); }
    public void backDown() { backKicker.setPosition(kickerBackDown); }

    public void resetIndexer() {
        rightDown();
        leftDown();
        backDown();
    }

    /* ================= ACTIONS ================= */

    public Action shootRight() {
        return new SequentialAction(
                new InstantAction(this::rightUp),
                new SleepAction(kickerSleep),
                new InstantAction(this::rightDown)
        );
    }

    public Action shootLeft() {
        return new SequentialAction(
                new InstantAction(this::leftUp),
                new SleepAction(kickerSleep),
                new InstantAction(this::leftDown)
        );
    }

    public Action shootBack() {
        return new SequentialAction(
                new InstantAction(this::backUp),
                new SleepAction(kickerSleep),
                new InstantAction(this::backDown)
        );
    }

    /* ================= DISTANCE ================= */

    public double safeDistance(RevColorSensorV3 s) {
        double d = s.getDistance(DistanceUnit.MM);
        return (Double.isNaN(d) || Double.isInfinite(d)) ? -1 : d;
    }

    private boolean ballPresent(RevColorSensorV3 a, RevColorSensorV3 b) {
        double da = safeDistance(a);
        double db = safeDistance(b);
        return (da > 0 && da < DISTANCE_THRESHOLD_MM)
                || (db > 0 && db < DISTANCE_THRESHOLD_MM);
    }

    /* ================= HUE ================= */

    public float getHue(RevColorSensorV3 sensor) {
        android.graphics.Color.RGBToHSV(
                sensor.red(), sensor.green(), sensor.blue(), hsv
        );
        return hsv[0];
    }

    private boolean isGreenHue(float h)  { return h > 160 && h < 180; }
    private boolean isPurpleHue(float h) { return h > 180 && h < 225; }

    private String classifyHue(float h) {
        if (isGreenHue(h))  return "GREEN";
        if (isPurpleHue(h)) return "PURPLE";
        return "UNKNOWN";
    }

    /* ================= FINAL COLOR ================= */

    public String getRightColor() {
        if (!ballPresent(colorRR, colorRL)) return "EMPTY";

        float h1 = getHue(colorRR);
        float h2 = getHue(colorRL);

        if (isGreenHue(h1) || isGreenHue(h2))   return "GREEN";
        if (isPurpleHue(h1) || isPurpleHue(h2)) return "PURPLE";

        return "UNKNOWN";
    }

    public String getLeftColor() {
        if (!ballPresent(colorLL, colorLR)) return "EMPTY";
        return classifyHue(getHue(colorLL)); // LL ONLY
    }

    public String getBackColor() {
        if (!ballPresent(colorBR, colorBL)) return "EMPTY";

        float h1 = getHue(colorBR);
        float h2 = getHue(colorBL);

        if (isGreenHue(h1) || isGreenHue(h2))   return "GREEN";
        if (isPurpleHue(h1) || isPurpleHue(h2)) return "PURPLE";

        return "UNKNOWN";
    }

    /* ================= BALL COUNT ================= */

    public int countBalls() {
        int balls = 0;
        if (ballPresent(colorRR, colorRL)) balls++;
        if (ballPresent(colorLL, colorLR)) balls++;
        if (ballPresent(colorBR, colorBL)) balls++;
        return balls;
    }

    /* ================= MOTIF SHOOT ================= */

    public void shootMotif() {
        String motif = motifPattern;

        java.util.Map<String, String> colors = new java.util.HashMap<>();
        colors.put("R", getRightColor());
        colors.put("L", getLeftColor());
        colors.put("B", getBackColor());

        for (int i = 0; i < motif.length(); i++) {
            String target = motif.charAt(i) == 'P' ? "PURPLE" : "GREEN";

            String spot = null;
            for (String s : colors.keySet()) {
                if (colors.get(s).equals(target)) {
                    spot = s;
                    break;
                }
            }

            if (spot != null) {
                switch (spot) {
                    case "R": rightUp(); break;
                    case "L": leftUp();  break;
                    case "B": backUp();  break;
                }

                sleepMillis((long)(kickerSleep * 1000));

                switch (spot) {
                    case "R": rightDown(); break;
                    case "L": leftDown();  break;
                    case "B": backDown();  break;
                }

                colors.put(spot, "USED");
                sleepMillis((long)(shootSleep * 1000));
            }
        }
    }

    private void sleepMillis(long ms) {
        try { Thread.sleep(ms); }
        catch (InterruptedException e) { Thread.currentThread().interrupt(); }
    }
}
