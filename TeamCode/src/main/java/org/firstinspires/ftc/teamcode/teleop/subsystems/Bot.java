package org.firstinspires.ftc.teamcode.teleop.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d   ;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.auto.tuning.MecanumDrive;

@Config
public class Bot {
    public static Bot instance;
    public OpMode opMode;

    public Turret turret;
    public Intake intake;
    public Lift lift;
    public Indexer indexer;
    public Screen screen;
    public Limelight limelight;

    public static Pose2d storedPose = new Pose2d(0, 0, 0);
    public static Pose2d resetPose = new Pose2d(-63, -61, Math.toRadians(-90));
    public static Vector2d goalPose = new Vector2d(62, 60); //initializes with blue, switches based on alliance
    public static Vector2d targetPose = goalPose;
    public Pose2d positionLockPose;
    public boolean shooting = false, sensorIntaking;

    public static MecanumDrive drive;
    public static double headingLockGain = 4.5, positionLockGain = 4.5;
    public boolean positionLockEnabled = false;

    public static enum allianceOptions {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }

    public static enum startingPosition {
        CLOSE,
        FAR
    }

    public static enum Motif {
        GPP,
        PGP,
        PPG,
        UNKNOWN //TODO Remove
    }

    public static enum HoodPosition {
        MID,
        FAR
    }

    public static Motif motif = Motif.PPG;
    public HoodPosition hoodPosition = HoodPosition.MID;

    private static allianceOptions alliance = allianceOptions.BLUE_ALLIANCE;
    private static startingPosition startingPos = startingPosition.FAR;

    private Bot(OpMode opMode) {
        this.opMode = opMode;

        drive = new MecanumDrive(opMode.hardwareMap, storedPose);
        limelight = new Limelight(opMode);
        turret = new Turret(opMode);
        intake = new Intake(opMode);
        lift = new Lift(opMode);
        indexer = new Indexer(opMode);
        screen = new Screen(opMode, this);
        updatePoses();

        setMidShooting();
    }

    public void switchAlliance() {
        if (isRed()) {
            setAllianceBlue();
        } else {
            setAllianceRed();
        }
    }

    public void setAllianceBlue() {
        alliance = allianceOptions.BLUE_ALLIANCE;
        limelight.trackBlueAlliance();
        updatePoses();
    }

    public void setAllianceRed() {
        alliance = allianceOptions.RED_ALLIANCE;
        limelight.trackRedAlliance();
        updatePoses();
    }

    public void setFar() {
        startingPos = startingPosition.FAR;
    }

    public void setClose() {
        startingPos = startingPosition.CLOSE;
    }

    public static void updatePoses() {
        if (isRed()) {
            goalPose = new Vector2d(goalPose.x, -1 * Math.abs(goalPose.y));
            resetPose = new Pose2d(resetPose.position.x, Math.abs(resetPose.position.y), Math.abs(resetPose.heading.log()));

        } else {
            goalPose = new Vector2d(goalPose.x, Math.abs(goalPose.y));
            resetPose = new Pose2d(resetPose.position.x, -1 * Math.abs(resetPose.position.y), -1 * Math.abs(resetPose.heading.log()));
        }
        targetPose = goalPose;
    }
//
//    public void setTargetFarAutoGoal() {
//        targetPose = farAutoGoalPose;
//    }

    public void setTargetGoalPose() {
        targetPose = goalPose;
    }

    public void resetPose() {
        drive.localizer.setPose(resetPose);
    }

    public static void useStoredPose() {
        drive.localizer.setPose(storedPose);
    }

    public static boolean isRed() {
        return alliance == allianceOptions.RED_ALLIANCE;
    }

    public static boolean isBlue() {
        return alliance == allianceOptions.BLUE_ALLIANCE;
    }

    public static allianceOptions getAlliance() {
        return alliance;
    }

    public static boolean isFar() {
        return startingPos == startingPosition.FAR;
    }

    public static boolean isClose() {
        return startingPos == startingPosition.CLOSE;
    }

    public static startingPosition getStartingPos() {
        return startingPos;
    }

    public void switchStartingPos() {
        if (startingPos == startingPosition.FAR) {
            startingPos = startingPosition.CLOSE;
        } else {
            startingPos = startingPosition.FAR;
        }
    }

    public void sensorIntake(boolean on) {
        sensorIntaking = on;
    }

    public void reverseIntake() {
        intake.reverse();
        sensorIntaking = false;
    }

    public void intake() {
        intake.intake();
        sensorIntaking = false;
    }

    public void stopIntake() {
        intake.stop();
        sensorIntaking = false;
    }

    public void enableFullAuto(boolean on) {
        turret.enableFullAuto(on);
    }

    public void enableShooter(boolean on) {
        turret.enableShooter(on);
    }

    public void setMidShooting() {
        // Hood angle and RPM are now set by distance-based interpolation in the turret.
        hoodPosition = HoodPosition.MID;
    }

    public void setFarShooting() {
        // Hood angle and RPM are now set by distance-based interpolation in the turret.
        hoodPosition = HoodPosition.FAR;
    }

    public void switchShooting() {
        switch(hoodPosition) {
            case MID:
                setFarShooting();
            case FAR:
                setMidShooting();
                break;
        }
    }


    public void driveRobotCentric(double forwardInput, double strafeInput, double turnInput, double driveSpeed) {
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(driveSpeed * forwardInput, driveSpeed * strafeInput),
                driveSpeed * turnInput));
    }

    public void driveHeadingLock(double forwardInput, double strafeInput, double driveSpeed) {
        double targetHeading = Math.toRadians(isRed() ? -45.0 : 45.0);
        double currentHeading = storedPose.heading.log();
        double headingError = normalizeRadians(targetHeading - currentHeading);
        double turn = headingError * headingLockGain;

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(driveSpeed * forwardInput, driveSpeed * strafeInput), turn));
    }

    public void drivePoseLock() {
        if (!positionLockEnabled) {
            positionLockPose = storedPose;
        }
        positionLockEnabled = true;

        Pose2d pose = storedPose;

        Vector2d posError = positionLockPose.position.minus(pose.position);
        double headingError = normalizeRadians(positionLockPose.heading.log() - pose.heading.log());

        Vector2d translation = posError.times(positionLockGain);
        double turn = headingError * headingLockGain;

        drive.setDrivePowers(
                new PoseVelocity2d(
                        translation,
                        turn
                )
        );
    }

    public Action enableShooter() {
        return new InstantAction(()-> enableShooter(true));
    }

    public Action disableShooter() {
        return new InstantAction(() -> enableShooter(false));
    }

    public void periodic() {
        clearBulkCache();
        indexer.updateSensorCache();
        limelight.periodic();
        turret.periodic();
        lift.periodic();
        screen.periodic();
        drive.updatePoseEstimate();
        if (sensorIntaking) {
            if (indexer.countBalls()==3) {
                intake.reverse();
            } else {
                intake.intake();
            }
        }
    }

    public void clearBulkCache() {
        drive.clearBulkCache();
    }

    public Action actionPeriodic() {
        return new actionPeriodic();
    }
    public class actionPeriodic implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            periodic();
            return true;
        }
    }

    private static double normalizeRadians(double angleRad) {
        return Math.atan2(Math.sin(angleRad), Math.cos(angleRad));
    }
//
//    public Action actionNoScreenPeriodic() {
//        return new actionPeriodic();
//    }
//    public class actionNoScreenPeriodic implements Action {
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            periodic();
//            return true;
//        }
//    }

    // get bot instance
    public static Bot getInstance() {
        if (instance == null) {
            throw new IllegalStateException("tried to getInstance of Bot when uninitialized!");
        }
        return instance;
    }

    public static Bot getInstance(OpMode opMode) {
        if (instance == null) {
            return instance = new Bot(opMode);
        }
        instance.opMode = opMode;
        return instance;
    }

}
