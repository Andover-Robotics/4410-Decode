package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
public class Limelight {
    private final Limelight3A limelight;
    public LLResult llResult;
    public static Pose3D llBotPose = new Pose3D(
            new Position(DistanceUnit.INCH, 0, 0, 0, 0),
            new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0)
    );

    public static double llxRLOffset = 120, llyRLOffset = 108.5;
    public static boolean obelisk = false;

    public Limelight(OpMode opMode) {
        limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
    }

    public void setPipeline(int i) {
        limelight.pipelineSwitch(i);
        /*
            0 is blue alliance
            1 is red alliance
            2 is obelisk tracking
         */
    }

    public void trackRedAlliance() {
        setPipeline(1);
        obelisk = false;
    }

    public void trackBlueAlliance() {
        setPipeline(0);
        obelisk = false;
    }

    public void trackAlliance() {
        if (Bot.getAlliance() == Bot.allianceOptions.BLUE_ALLIANCE) {
            trackBlueAlliance();
        } else {
            trackRedAlliance();
        }
    }

    public void trackObelisk() {
        setPipeline(2);
        obelisk = true;
    }

    public void setObelisk(boolean enable) {
        obelisk = enable;
    }

    public boolean isObelisk() {
        return obelisk;
    }

    public void periodic() {
        llResult = limelight.getLatestResult();

        if (!obelisk) {
            limelight.updateRobotOrientation(Math.toDegrees(Bot.storedPose.heading.log()));
            if (llResult != null && llResult.isValid()) {
                llBotPose = llResult.getBotpose_MT2();
            }
        } else {
            if (llResult != null && llResult.isValid()
                    && llResult.getFiducialResults() != null
                    && !llResult.getFiducialResults().isEmpty()) {
                int id = llResult.getFiducialResults().get(0).getFiducialId();
                if (id == 21) {
                    Bot.motif = Bot.Motif.GPP;
                } else if (id == 22) {
                    Bot.motif = Bot.Motif.PGP;
                } else if (id == 23) {
                    Bot.motif = Bot.Motif.PPG;
                }
            }
        }
    }

    public void relocalizeBotPose() {
//        Bot.drive.localizer.setPose(new Pose2d(
//                llBotPose.getPosition().toUnit(DistanceUnit.INCH).x + llxRLOffset,
//                llBotPose.getPosition().toUnit(DistanceUnit.INCH).y + llyRLOffset,
//                Math.toRadians(llBotPose.getOrientation().getYaw())
//        ));
    }
}