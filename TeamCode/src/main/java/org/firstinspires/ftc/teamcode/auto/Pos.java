package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

@Config
public class Pos {
    // SOME VARIABLES
    public static final double GATE_INTAKE_ANGLE = Math.toRadians(60);

    // INITIAL
    public static Pose2d initialFarBluePose = new Pose2d(-63, 9, Math.toRadians(0));
    public static Pose2d initialCloseBluePose = new Pose2d(60, 41, Math.toRadians(0));
    public static Pose2d initialFarRedPose = transformRed(initialFarBluePose);
    public static Pose2d initialCloseRedPose = transformRed(initialCloseBluePose);

    // INTAKE
    public static Pose2d blueMidIntake = new Pose2d(-14, 29, Math.toRadians(90));

    public static Pose2d gate = new Pose2d(-14.5, 61, GATE_INTAKE_ANGLE);
    public static Pose2d gateIntake = new Pose2d(-12, 60, GATE_INTAKE_ANGLE);

    public static Pose2d blueCloseIntake = new Pose2d(13, 29, Math.toRadians(90));

    public static Pose2d blueFarIntake = new Pose2d(-35, 29, Math.toRadians(90));

    public static Pose2d blueHpIntake = new Pose2d(-63, 60.5, Math.toRadians(179));

    public static Vector2d blueHpIntakeInter = new Vector2d(-58, 24);

    public static Pose2d blueSecretTunnel = new Pose2d(-40, 59, Math.toRadians(135));

    // SHOOTING (Vectors, as we do not care about robot orientation here)
    public static Vector2d closeShoot = new Vector2d(7, 15);
    public static Vector2d park = new Vector2d(-1, 21);
    public static Vector2d farShoot = new Vector2d(-59, 9);

    public static Pose2d transformRed(Pose2d pose) {
        return new Pose2d(new Vector2d(pose.position.x, -pose.position.y), -pose.heading.log());
    }
}