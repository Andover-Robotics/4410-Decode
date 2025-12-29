package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Indexer {

    //just for now bc no limelight
    public String motifPattern="PPG";
    //to set which motif in future
//    if (idtag2x) {
//        motifPattern="PPG";
//
//    } else if (idtag2y){
//        motifPattern="PGP";
//    }else if (iftag2z){
//        motifPattern="GPP";
//    }



    //kicker servos
    public Servo leftKicker;
    public Servo rightKicker;
    public Servo backKicker;

    //kicker values
    public static double kickerLeftDown = 0.7;
    public static double kickerLeftUp = 0.45;
    public static double kickerRightDown = 0.63;
    public static double kickerRightUp = 0.37;
    public static double kickerBackDown = 0.60;
    public static double kickerBackUp = 0.34;

    public static double kickerSleep=0.5;
    public static double shootSleep=0.5;

    //color sensors
    public RevColorSensorV3 colorRR;//dont use for distance
    public RevColorSensorV3 colorRL;//dont use for hue
    public RevColorSensorV3 colorLL;
    public RevColorSensorV3 colorLR;
    public RevColorSensorV3 colorBL;//dont use for distance
    public RevColorSensorV3 colorBR;

    //sensing variables
    public float hueGreen;
    public float huePurple;
    public float hueNone;

    int gain=20;

    // distance threshold
    static final double DISTANCE_THRESHOLD_MM = 28.0;
    private float[] hsv = new float[3];



    //hardware
    public Indexer (OpMode opMode) {
        leftKicker= opMode.hardwareMap.servo.get("leftKicker");
        rightKicker= opMode.hardwareMap.servo.get("rightKicker");
        backKicker= opMode.hardwareMap.servo.get("backKicker");

        colorRR= opMode.hardwareMap.get(RevColorSensorV3.class, "colorRR");
        colorRR.setGain(gain);

        colorRL= opMode.hardwareMap.get(RevColorSensorV3.class, "colorRL");
        colorRL.setGain(gain);

        colorLL= opMode.hardwareMap.get(RevColorSensorV3.class, "colorLL");
        colorLL.setGain(gain);

        colorLR= opMode.hardwareMap.get(RevColorSensorV3.class, "colorLR");
        colorLR.setGain(gain);

        colorBR= opMode.hardwareMap.get(RevColorSensorV3.class, "colorBR");
        colorBR.setGain(gain);

        colorBL= opMode.hardwareMap.get(RevColorSensorV3.class, "colorBL");
        colorBL.setGain(gain);

    }

    // kicker methods
    public void rightDown() {
        rightKicker.setPosition(kickerRightDown);
    }
    public void rightUp() {
        rightKicker.setPosition(kickerRightUp);
    }
    public void leftDown() {
        leftKicker.setPosition(kickerLeftDown);
    }
    public void leftUp() {
        leftKicker.setPosition(kickerLeftUp);
    }
    public void backDown() {
        backKicker.setPosition(kickerBackDown);
    }
    public void backUp() {
        backKicker.setPosition(kickerBackUp);
    }

    public Action shootBack() {
        return new SequentialAction(
                new InstantAction(() -> backUp()),
                new SleepAction(kickerSleep),
                new InstantAction(() -> backDown()),
                new SleepAction(0.1)
        );
    }
    public Action shootRight() {
        return new SequentialAction(
                new InstantAction(() -> rightUp()),
                new SleepAction(kickerSleep),
                new InstantAction(() -> rightDown()),
                new SleepAction(0.1)
        );
    }
    public Action shootLeft() {
        return new SequentialAction(
                new InstantAction(() -> leftUp()),
                new SleepAction(kickerSleep),
                new InstantAction(() -> leftDown()),
                new SleepAction(0.1)
        );
    }

    public Action shootLRB() {
        return new SequentialAction(
                new InstantAction(() -> leftUp()),
                new SleepAction(kickerSleep),
                new InstantAction(() -> leftDown()),
                new SleepAction(shootSleep),
                new InstantAction(() -> rightUp()),
                new SleepAction(kickerSleep),
                new InstantAction(() -> rightDown()),
                new SleepAction(shootSleep),
                new InstantAction(() -> backUp()),
                new SleepAction(kickerSleep),
                new InstantAction(() -> backDown()),
                new SleepAction(0.1)
        );
    }
    public Action shootLBR() {
        return new SequentialAction(
                new InstantAction(() -> leftUp()),
                new SleepAction(kickerSleep),
                new InstantAction(() -> leftDown()),
                new SleepAction(shootSleep),
                new InstantAction(() -> backUp()),
                new SleepAction(kickerSleep),
                new InstantAction(() -> backDown()),
                new SleepAction(shootSleep),
                new InstantAction(() -> rightUp()),
                new SleepAction(kickerSleep),
                new InstantAction(() -> rightDown()),
                new SleepAction(0.1)
        );
    }
    public Action shootBLR() {
        return new SequentialAction(
                new InstantAction(() -> backUp()),
                new SleepAction(kickerSleep),
                new InstantAction(() -> backDown()),
                new SleepAction(shootSleep),
                new InstantAction(() -> leftUp()),
                new SleepAction(kickerSleep),
                new InstantAction(() -> leftDown()),
                new SleepAction(shootSleep),
                new InstantAction(() -> rightUp()),
                new SleepAction(kickerSleep),
                new InstantAction(() -> rightDown()),
                new SleepAction(0.1)
        );
    }
    public Action shootBRL() {
        return new SequentialAction(
                new InstantAction(() -> backUp()),
                new SleepAction(kickerSleep),
                new InstantAction(() -> backDown()),
                new SleepAction(shootSleep),
                new InstantAction(() -> rightUp()),
                new SleepAction(kickerSleep),
                new InstantAction(() -> rightDown()),
                new SleepAction(shootSleep),
                new InstantAction(() -> leftUp()),
                new SleepAction(kickerSleep),
                new InstantAction(() -> leftDown()),
                new SleepAction(0.1)
        );
    }

    public Action shootRBL() {
        return new SequentialAction(
                new InstantAction(() -> rightUp()),
                new SleepAction(kickerSleep),
                new InstantAction(() -> rightDown()),
                new SleepAction(shootSleep),
                new InstantAction(() -> backUp()),
                new SleepAction(kickerSleep),
                new InstantAction(() -> backDown()),
                new SleepAction(shootSleep),
                new InstantAction(() -> leftUp()),
                new SleepAction(kickerSleep),
                new InstantAction(() -> leftDown()),
                new SleepAction(0.1)
        );
    }

    public Action shootRLB() {
        return new SequentialAction(
                new InstantAction(() -> rightUp()),
                new SleepAction(kickerSleep),
                new InstantAction(() -> rightDown()),
                new SleepAction(shootSleep),
                new InstantAction(() -> leftUp()),
                new SleepAction(kickerSleep),
                new InstantAction(() -> leftDown()),
                new SleepAction(shootSleep),
                new InstantAction(() -> backUp()),
                new SleepAction(kickerSleep),
                new InstantAction(() -> backDown()),
                new SleepAction(0.1)
        );
    }

    public void resetIndexer() {
        rightKicker.setPosition(kickerRightDown);
        leftKicker.setPosition(kickerLeftDown);
        backKicker.setPosition(kickerBackDown);
    }
    // ===== distance helpers =====
    public double safeDistance(RevColorSensorV3 sensor) {
        double d = sensor.getDistance(DistanceUnit.MM);
        if (Double.isNaN(d) || Double.isInfinite(d)) return -1;
        return d;
    }

    private boolean isBallPresent(RevColorSensorV3 distanceSensor) {
        double dist = safeDistance(distanceSensor);
        return dist > 0 && dist < DISTANCE_THRESHOLD_MM;
    }

    // ===== hue helpers =====
    public float getHue(RevColorSensorV3 sensor) {
        android.graphics.Color.RGBToHSV(sensor.red(), sensor.green(), sensor.blue(), hsv);
        return hsv[0];
    }

    private boolean isGreenHue(float h) { return h > 160 && h < 180; }
    private boolean isPurpleHue(float h) { return h > 180 && h < 225; }

    // ===== final color getters =====
    public String getRightColor() {
        if (!isBallPresent(colorRL)) return "EMPTY";
        float h = getHue(colorRR);
        if (isGreenHue(h)) return "GREEN";
        if (isPurpleHue(h)) return "PURPLE";
        return "UNKNOWN";
    }

    public String getLeftColor() {
        if (!isBallPresent(colorLR))  return "EMPTY";
        if (!isBallPresent(colorLL))  return "EMPTY";
        float h = getHue(colorLR);
        float hh =getHue(colorLL);
        if (isBallPresent(colorLL) && isBallPresent(colorLR)) {
            if (isGreenHue(h)) return "GREEN";
            if (isPurpleHue(h)) return "PURPLE";
        } else if (isBallPresent(colorLR)){
            if (isGreenHue(h)) return "GREEN";
            if (isPurpleHue(h)) return "PURPLE";
        } else if (isBallPresent(colorRR)){
            if (isGreenHue(hh)) return "GREEN";
            if (isPurpleHue(hh)) return "PURPLE";
        }
        return "UNKNOWN";
    }

    public String getBackColor() {
        if (!isBallPresent(colorBR)) return "EMPTY"; // use Back B for distance
        float h = getHue(colorBR); // use Back A for hue
        if (isGreenHue(h)) return "GREEN";
        if (isPurpleHue(h)) return "PURPLE";
        return "UNKNOWN";
    }
    public int countBalls() {
        int balls = 0;

        // Right spot
        if (isBallPresent(colorRL)) balls++;

        // Left spot
        if (isBallPresent(colorLR)) balls++;

        // Back spot (use Back B for distance)
        if (isBallPresent(colorBR)) balls++;

        return balls;
    }
    // In your Indexer class, add a "blocking" shoot method:
    public void shootMotifDirect() {
        // Copy the motif pattern
        String motif = motifPattern;

        // Get current colors
        java.util.Map<String, String> spotColors = new java.util.HashMap<>();
        spotColors.put("R", getRightColor());
        spotColors.put("L", getLeftColor());
        spotColors.put("B", getBackColor());

        // Iterate through motif
        for (int i = 0; i < motif.length(); i++) {
            char targetColorChar = motif.charAt(i);
            String targetColor = (targetColorChar == 'P') ? "PURPLE" : "GREEN";

            // Find first spot with that color
            String spotToShoot = null;
            for (String spot : spotColors.keySet()) {
                if (spotColors.get(spot).equalsIgnoreCase(targetColor)) {
                    spotToShoot = spot;
                    break;
                }
            }

            // Shoot the chosen spot
            if (spotToShoot != null) {
                switch (spotToShoot) {
                    case "R": rightUp(); sleepMillis((long)(kickerSleep*1000)); rightDown(); break;
                    case "L": leftUp(); sleepMillis((long)(kickerSleep*1000)); leftDown(); break;
                    case "B": backUp(); sleepMillis((long)(kickerSleep*1000)); backDown(); break;
                }

                // Mark spot as used
                spotColors.put(spotToShoot, "USED");
                // Optional: small pause between balls
                sleepMillis((long)(shootSleep*1000));
            }
        }
    }

    // Simple helper for sleeping in LinearOpMode
    private void sleepMillis(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }



}



