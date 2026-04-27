package org.firstinspires.ftc.teamcode.teleop.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d   ;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.auto.tuning.MecanumDrive;

@Config
public class Bot {
    public static Bot instance;
    public OpMode opMode;

    public Turret turret;
    public Intake intake;
    public Indexer indexer;
    public Screen screen;
    public Limelight limelight;
//    public static VoltageSensor voltageSensor;

    public static Pose2d storedPose = new Pose2d(0, 0, 0);
    public static Pose2d resetPose = new Pose2d(-63, -61, Math.toRadians(-90));
    public static Vector2d gateResetPose = new Vector2d(9, 54);
    public static Vector2d obeliskPose = new Vector2d(66, 0);
    public static Vector2d farRampPose = new Vector2d(33, 80);
    public static Vector2d hpRampPose = new Vector2d(38, 82);
    public static Vector2d goalPose = new Vector2d(62, 61); //initializes with blue, switches based on alliance
    public static Vector2d targetPose = goalPose;
    public Pose2d positionLockPose;
    public boolean shooting = false, sensorIntaking = true;
    private boolean screenPeriodicEnabled = true;
    public boolean actionsRunning = false;
    public static boolean unjamming = false, dontDoAgain = false;
    private double bulkCacheMs = 0;
    private double sensorCacheMs = 0;
    private double limelightMs = 0;
    private double turretMs = 0;
    private double intakeMs = 0;
    private double screenMs = 0;
    private double drivePoseMs = 0;
    private double periodicTotalMs = 0;

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
        turret = new Turret(opMode, drive.srsHubs);
        intake = new Intake(opMode);
        indexer = new Indexer(opMode, drive.srsHubs);
        screen = new Screen(opMode, this);
//        voltageSensor = opMode.hardwareMap.voltageSensor.iterator().next();
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
            obeliskPose = new Vector2d(obeliskPose.x, -1 * Math.abs(obeliskPose.y));
            resetPose = new Pose2d(resetPose.position.x, Math.abs(resetPose.position.y), Math.abs(resetPose.heading.log()));
            farRampPose = new Vector2d(farRampPose.x, -1 * Math.abs(farRampPose.y));
            gateResetPose = new Vector2d(gateResetPose.x, -1 * Math.abs(gateResetPose.y));
            hpRampPose = new Vector2d(hpRampPose.x, -1 * Math.abs(hpRampPose.y));

        } else {
            goalPose = new Vector2d(goalPose.x, Math.abs(goalPose.y));
            obeliskPose = new Vector2d(obeliskPose.x, Math.abs(obeliskPose.y));
            resetPose = new Pose2d(resetPose.position.x, -1 * Math.abs(resetPose.position.y), -1 * Math.abs(resetPose.heading.log()));
            farRampPose = new Vector2d(farRampPose.x, Math.abs(farRampPose.y));
            gateResetPose = new Vector2d(gateResetPose.x, Math.abs(gateResetPose.y));
            hpRampPose = new Vector2d(hpRampPose.x, Math.abs(hpRampPose.y));
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

    public void trackObelisk() {
        targetPose = obeliskPose;
        limelight.trackObelisk();
    }

    public void setTargetHpRampPose() {
        targetPose = hpRampPose;
    }

    public void setTargetFarRampPose() {
        targetPose = farRampPose;
    }

    public Action trackFarRampPose() {
        return new InstantAction(this::setTargetFarRampPose);
    }

    public Action trackHpRampPose() {
        return new InstantAction(this::setTargetHpRampPose);
    }

    public Action offsetMotifByRampArtifacts() {
        return new InstantAction(() -> {
            int artifacts = limelight.getLastDetectedArtifacts();
            indexer.rampDetectionOffsetAutoMotifBy(artifacts);
        });
    }

    public void resetPose() {
        drive.localizer.setPose(resetPose);
    }

    public void resetXY() {
        drive.localizer.setPose(new Pose2d(resetPose.position, drive.localizer.getPose().heading));
    }

    public void resetGateXY() {
        drive.localizer.setPose(new Pose2d(gateResetPose, drive.localizer.getPose().heading));
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

    public SequentialAction clearKickerJam() {
        actionsRunning = true;
        return new SequentialAction(
                new InstantAction(() -> indexer.rightTooHigh()),
                new SleepAction(0.3),
                new InstantAction(() -> intake.intake()),
                new SleepAction(1.3),
                new InstantAction(() -> indexer.rightReset()),
                new SleepAction(0.5),
                new InstantAction(() -> intake.reverse()),
                new SleepAction(0.6),
                indexer.jiggleKickers(),
                indexer.jiggleKickers(),
                new InstantAction(() -> actionsRunning = false)
        );
    }

    public SequentialAction clearIntakeJam() {
        actionsRunning = true;
        unjamming = true;
//        stopIntake();
        return new SequentialAction(
                new InstantAction(() -> intake.reverse()),
                new SleepAction(0.3),
                indexer.jiggleKickers(),
                new InstantAction(() -> unjamming = false),
                new InstantAction(() -> actionsRunning = false)
        );
    }

    public void periodic() {
        long loopStartNs = System.nanoTime();

        long sectionStartNs = loopStartNs;
//        clearBulkCache(); drive.updatePoseEstimate does this
        bulkCacheMs = nanosToMillis(System.nanoTime() - sectionStartNs);

        sectionStartNs = System.nanoTime();
        drive.srsHubs.update();
        sensorCacheMs = nanosToMillis(System.nanoTime() - sectionStartNs);

        sectionStartNs = System.nanoTime();
        indexer.updateSensorCache();
        sensorCacheMs += nanosToMillis(System.nanoTime() - sectionStartNs);

        sectionStartNs = System.nanoTime();
//        limelight.periodic();
        limelightMs = nanosToMillis(System.nanoTime() - sectionStartNs);

        sectionStartNs = System.nanoTime();
        turret.periodic();
        turretMs = nanosToMillis(System.nanoTime() - sectionStartNs);

        sectionStartNs = System.nanoTime();
        intake.periodic();
        intakeMs = nanosToMillis(System.nanoTime() - sectionStartNs);

        screenMs = 0;
        if (screenPeriodicEnabled) {
            sectionStartNs = System.nanoTime();
            screen.periodic();
            screenMs = nanosToMillis(System.nanoTime() - sectionStartNs);
        }

        sectionStartNs = System.nanoTime();
        drive.updatePoseEstimate();
        drivePoseMs = nanosToMillis(System.nanoTime() - sectionStartNs);
        periodicTotalMs = nanosToMillis(System.nanoTime() - loopStartNs);
    }

    public void autoPeriodic() {
        drive.srsHubs.update();
        indexer.updateSensorCache();
        limelight.periodic();
        turret.periodic();
        drive.updatePoseEstimate();
        screen.periodic();
        if (sensorIntaking) {
            if (indexer.countBalls()==3) {
                intake.reverse();
            } else {
                intake.intake();
            }
        }
    }


    public void teleopReverseIntake() {
        intake.reverse();
    }

    public void teleopIntake() {
        intake.intake();
    }

    public void teleopStopIntake() {
        intake.stop();
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

    public boolean isScreenPeriodicEnabled() {
        return screenPeriodicEnabled;
    }

    public double getBulkCacheMs() {
        return bulkCacheMs;
    }

    public double getSensorCacheMs() {
        return sensorCacheMs;
    }

    public double getLimelightMs() {
        return limelightMs;
    }

    public double getTurretMs() {
        return turretMs;
    }

    public double getIntakeMs() {
        return intakeMs;
    }

    public double getScreenMs() {
        return screenMs;
    }

    public double getDrivePoseMs() {
        return drivePoseMs;
    }

    public double getPeriodicTotalMs() {
        return periodicTotalMs;
    }

    public void setScreenPeriodicEnabled(boolean enabled) {
        screenPeriodicEnabled = enabled;
    }

    public void toggleScreenPeriodic() {
        screenPeriodicEnabled = !screenPeriodicEnabled;
    }

    public void checkBotPose(){
        //takes dy and dx of ll pose and current bot pose, and sees if delta is >4 inches
        Pose2d pose = storedPose;
        Pose2d llPose = pose3D2pose2D(Limelight.llBotPose);

        double dx = llPose.position.x - pose.position.x;
        double dy = llPose.position.y-pose.position.y;
        if(Math.abs(dx)>=4 || Math.abs(dy)>=4){
            limelight.relocalizeBotPose();
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

    public void clearBulkCache() {
        drive.clearBulkCache();
    }

    public Action actionPeriodic() {
        return new actionPeriodic();
    }
    public class actionPeriodic implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            autoPeriodic();
            return true;
        }
    }

    private static double normalizeRadians(double angleRad) {
        return Math.atan2(Math.sin(angleRad), Math.cos(angleRad));
    }

    public static double getBatteryVoltage() {
        return drive.voltageSensor.getVoltage();
//        return 13;
    }

    public static Pose2d pose3D2pose2D(Pose3D pose){
        double x = pose.getPosition().toUnit(DistanceUnit.INCH).x;
        double y = pose.getPosition().toUnit(DistanceUnit.INCH).y;
        double heading = Math.toRadians(pose.getOrientation().getYaw());
        return new Pose2d(x,y,heading);
    }

    private double nanosToMillis(long nanos) {
        return nanos / 1_000_000.0;
    }




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
