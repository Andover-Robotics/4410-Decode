package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

@Config
public class Pos {
    // SOME VARIABLES
    public static final double GATE_INTAKE_ANGLE = Math.toRadians(57);

    // INITIAL
    public static Pose2d initialFarBluePose = new Pose2d(-63, 9, Math.toRadians(0));
    public static Pose2d initialCloseBluePose = new Pose2d(62, 41, Math.toRadians(180));
    public static Pose2d initialFarRedPose = transformRed(initialFarBluePose);
    public static Pose2d initialCloseRedPose = transformRed(initialCloseBluePose);

    // INTAKE

    public static Pose2d blueMidIntake = new Pose2d(-11, 34.5, Math.toRadians(90));
    public static double intakeDisp = 23;

    public static Pose2d gate = new Pose2d(-11, 67, GATE_INTAKE_ANGLE);

    public static Pose2d gateSideOpenHeadOn = new Pose2d(1, 62, Math.toRadians(90));
    public static Pose2d gateSideOpenHeadOnBack = new Pose2d(1, gateSideOpenHeadOn.position.y - 20, Math.toRadians(90));

    public static Pose2d blueCloseIntake = new Pose2d(11, 32, Math.toRadians(90));
    public static double closeIntake = 15;

    public static Pose2d blueFarIntakeFarAuto = new Pose2d(-44, 63, Math.toRadians(90));
    public static Pose2d blueFarIntakeCloseAuto = new Pose2d(-33, 35, Math.toRadians(90));

    public static Pose2d pushPark = new Pose2d(-58,  20, Math.toRadians(90));

    public static Pose2d blueHpIntake = new Pose2d(-44, 68, Math.toRadians(179));
    public static Pose2d redHpIntake = new Pose2d(-44, 62, Math.toRadians(179));
    public static Pose2d blueHpSideIntake = new Pose2d(-61, 60, Math.toRadians(90));

    public static Pose2d blueSecretTunnel = new Pose2d(-36, 62, Math.toRadians(35));
    public static Pose2d blueHpCycle = new Pose2d(-62, 55, Math.toRadians(90));

    // SHOOTING (Vectors, as we do not care about robot orientation here)
    public static Vector2d firstShoot = new Vector2d(18, 20);
    public static Vector2d closeShoot = new Vector2d(4, 17);
    public static Vector2d closeShootPark = new Vector2d(30, 18);
    public static Vector2d closeGateCycleShoot = new Vector2d(4, 19);
    public static Vector2d closePark = new Vector2d(-1, 21);
    public static Vector2d farShoot = new Vector2d(-53, 19);
    public static Vector2d farPark = new Vector2d(-40, 24);

    public static Pose2d transformRed(Pose2d pose) {
        return new Pose2d(new Vector2d(pose.position.x, -pose.position.y), -pose.heading.log());
    }
}