package org.firstinspires.ftc.teamcode.teleop.subsystems;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@Config
public class Indexer {

    public enum Motif {
        PPG,
        GPP,
        PGP
    }
    String currentMotif=Motif.PPG.name();

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


    //color sensors
    public RevColorSensorV3 colorRR;
    public RevColorSensorV3 colorRL;
    public RevColorSensorV3 colorLL;
    public RevColorSensorV3 colorLR;
    public RevColorSensorV3 colorBL;
    public RevColorSensorV3 colorBR;

    //sensing variables
    public float hueGreen;
    public float huePurple;
    public float hueNone;

    int gain=20;


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
                new SleepAction(0.5),
                new InstantAction(() -> backDown()),
                new SleepAction(0.1)
        );
    }
    public Action shootRight() {
        return new SequentialAction(
                new InstantAction(() -> rightUp()),
                new SleepAction(0.5),
                new InstantAction(() -> rightDown()),
                new SleepAction(0.1)
        );
    }
    public Action shootLeft() {
        return new SequentialAction(
                new InstantAction(() -> leftUp()),
                new SleepAction(0.5),
                new InstantAction(() -> leftDown()),
                new SleepAction(0.1)
        );
    }










    public void resetIndexer() {
        rightKicker.setPosition(kickerRightDown);
        leftKicker.setPosition(kickerLeftDown);
        backKicker.setPosition(kickerBackDown);
    }


    //color sensing methods
    public boolean isGreen(RevColorSensorV3 cs){
        NormalizedRGBA colorSensor =cs.getNormalizedColors();

        hueGreen = JavaUtil.colorToHue(colorSensor.toColor());

        return hueGreen < 200 && hueGreen > 100;
    }

    public boolean isPurple(RevColorSensorV3 cs) {
        NormalizedRGBA colorSensor = cs.getNormalizedColors();

        huePurple = JavaUtil.colorToHue(colorSensor.toColor());

        return (huePurple > 250) && (huePurple < 310);
    }


    public boolean isNone(RevColorSensorV3 cs) {
        NormalizedRGBA colorSensor = cs.getNormalizedColors();

        hueNone = JavaUtil.colorToHue(colorSensor.toColor());

        return !((hueGreen >250) && (hueGreen <310)) && !((hueGreen < 200) && (hueGreen > 100));
    }

    public float getHue(RevColorSensorV3 cs) {
        NormalizedRGBA colorSensor = cs.getNormalizedColors();
        return JavaUtil.colorToHue(colorSensor.toColor());
    }

//    public boolean isGreenLeft(RevColorSensorV3 cs){
//        NormalizedRGBA leftColor1 =cs.getNormalizedColors();
//        NormalizedRGBA leftColor2 =cs.getNormalizedColors();
//
//        hueLeft = JavaUtil.colorToHue(leftColor1.toColor());
//
//        return hueLeft < 200 && hueLeft > 100;
//    }
//
//    public boolean isPurpleLeft(RevColorSensorV3 cs) {
//        NormalizedRGBA leftColor1 = cs.getNormalizedColors();
//        NormalizedRGBA leftColor2 =cs.getNormalizedColors();
//
//        hueRight = JavaUtil.colorToHue(leftColor1.toColor());
//
//        return (hueLeft > 250) && (hueLeft < 310);
//    }
//
//    public boolean isNoneLeft(RevColorSensorV3 cs) {
//        NormalizedRGBA leftColor1 = cs.getNormalizedColors();
//        NormalizedRGBA leftColor2 =cs.getNormalizedColors();
//
//        hueRight = JavaUtil.colorToHue(leftColor1.toColor());
//
//        return !((hueLeft>250) && (hueLeft<310)) && !((hueLeft < 200) && (hueLeft > 100));
//    }
//    public boolean isGreenBack(RevColorSensorV3 cs){
//        NormalizedRGBA backColor1 =cs.getNormalizedColors();
//        NormalizedRGBA backColor2 =cs.getNormalizedColors();
//
//        hueRight = JavaUtil.colorToHue(backColor1.toColor());
//
//        return hueBack < 200 && hueBack > 100;
//    }
//
//    public boolean isPurpleBack(RevColorSensorV3 cs) {
//        NormalizedRGBA backColor1 = cs.getNormalizedColors();
//        NormalizedRGBA backColor2 =cs.getNormalizedColors();
//
//        hueRight = JavaUtil.colorToHue(backColor1.toColor());
//
//        return (hueBack > 250) && (hueBack < 310);
//    }
//
//    public boolean isNoneBack(RevColorSensorV3 cs) {
//        NormalizedRGBA backColor1 = cs.getNormalizedColors();
//        NormalizedRGBA backColor2 =cs.getNormalizedColors();
//
//        hueRight = JavaUtil.colorToHue(backColor1.toColor());
//
//        return !((hueBack>250) && (hueBack<310)) && !((hueBack < 200) && (hueBack > 100));
//    }


}


