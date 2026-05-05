package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.ArrayDeque;
import java.util.Deque;
import java.util.List;

@Config
public class Limelight {
    private final Limelight3A limelight;
    public LLResult llResult;
    public static Pose2d transformedBotPose = new Pose2d(0, 0, 0);
    public static Pose2d llBotPose2d;
    public static Pose3D llBotPose = new Pose3D(
            new Position(DistanceUnit.INCH, 0, 0, 0, 0),
            new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0)
    );

    public static double llxRLOffset = 0, llyRLOffset = 0;
    public static boolean obelisk = false;
    public double headingInput;
    public static double llxoffset=0,llyoffset=0;
    public static int lastDetectedArtifacts = 0, farArtifacts = 0, hpArtifacts = 0;
    private static final int BLUE_RAMP_PIPELINE_INDEX = 5;
    private static final int RED_RAMP_PIPELINE_INDEX = 6;
    private static final int MIN_ARTIFACT_SIDE_PIXELS = 5;
    private static final int ROLLING_WINDOW_SIZE = 1;
    public static double taThreshold = 0.0025;
    public static final Deque<Integer> artifactHistory = new ArrayDeque<>();


    public Limelight(OpMode opMode) {
        limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
    }

    private void setPipeline(int i) {
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

    public void trackBlueRamp() {
        setPipeline(BLUE_RAMP_PIPELINE_INDEX);
        obelisk = false;
    }

    public void trackRedRamp() {
        setPipeline(RED_RAMP_PIPELINE_INDEX);
        obelisk = false;
    }

    public void trackRampAlliance() {
        if (Bot.getAlliance() == Bot.allianceOptions.BLUE_ALLIANCE) {
            trackBlueRamp();
        } else {
            trackRedRamp();
        }
    }


    public int calculateRollingAverageRamp(LLResult latestResult) {
        int validArtifacts = 0;

        if (latestResult != null && latestResult.isValid()) {
            List<LLResultTypes.DetectorResult> detectorResults = latestResult.getDetectorResults();
            if (detectorResults != null) {
                validArtifacts = detectorResults.size();
//                for (LLResultTypes.DetectorResult detectorResult : detectorResults) {
//                    if (detectorResult.getTargetArea() > taThreshold) {
//                        validArtifacts++;
//                    }
//                }
            }
        }

        artifactHistory.addLast(validArtifacts);
        if (artifactHistory.size() > ROLLING_WINDOW_SIZE) {
            artifactHistory.removeFirst();
        }

        if (artifactHistory.isEmpty()) {
            return 0;
        }

        int sum = 0;
        for (int sample : artifactHistory) {
            sum += sample;
        }
        lastDetectedArtifacts = (int) Math.round((double) sum / artifactHistory.size());
        return lastDetectedArtifacts;
    }

    public int getLastDetectedArtifacts() {
        return lastDetectedArtifacts;
    }

    public void saveFarArtifacts() {
        farArtifacts = lastDetectedArtifacts;
    }

    public void saveHpArtifacts() {
        hpArtifacts = lastDetectedArtifacts;
    }

    public void takeSnapshot(String name) {
        limelight.captureSnapshot(name);
    }

    public void setObelisk(boolean enable) {
        obelisk = enable;
    }

    public boolean isObelisk() {
        return obelisk;
    }

    public void periodic() {

        llResult = limelight.getLatestResult();
        calculateRollingAverageRamp(llResult);
        headingInput = Math.toDegrees(Bot.drive.localizer.getPose().heading.log()) - Turret.currentPosDegs + 180;

        if (!obelisk) {
            if(Bot.getAlliance() == Bot.allianceOptions.RED_ALLIANCE){
                llxoffset = -7;
                llyoffset = -1;
            } else if (Bot.getAlliance() == Bot.allianceOptions.BLUE_ALLIANCE) {
                llxoffset = -4;
                llyoffset = -6;

            }
            // when not looking at the obelisk (add condition for when tracking specific goals?) get robot pos using turret pos and current heading
            limelight.updateRobotOrientation(headingInput);
            if (llResult != null && llResult.isValid()) {
                llBotPose = llResult.getBotpose_MT2();
                llBotPose2d = Bot.pose3D2pose2D(llBotPose);
                transformedBotPose = new Pose2d(-(llxoffset+llBotPose2d.position.x),-(llyoffset+llBotPose2d.position.y),Bot.drive.localizer.getPose().heading.log());
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
        Bot.drive.localizer.setPose(transformedBotPose);
    }
}
