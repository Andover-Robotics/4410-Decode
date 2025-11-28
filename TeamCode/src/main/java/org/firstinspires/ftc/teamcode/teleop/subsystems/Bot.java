package org.firstinspires.ftc.teamcode.teleop.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d   ;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.auto.MecanumDrive;

@Config
public class Bot {
    public static Bot instance;
    public OpMode opMode;

    public Turret turret;
    public Intake intake;
    public Lift lift;

    public static Pose2d storedPose = new Pose2d(0, 0, 0);
    public static Pose2d resetPose = new Pose2d(-61, -63, Math.toRadians(-90));
    public static Vector2d goalPose = new Vector2d(67, 60); //initializes with blue, switches based on alliance
    public static Vector2d farAutoGoalPose = new Vector2d(61, 64);
    public static Vector2d targetPose = goalPose;
    public static double shootTime = 0.3, autoFarShootDeley = 0.4, shootDelay = 0.4, shootDelayCF = 0.0024, shootDelayRPMThreshold = 3900;
    public boolean shooting = false;

    public static MecanumDrive drive;

    public static enum allianceOptions {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }

    public static enum startingPosition {
        CLOSE,
        FAR
    }

    private static allianceOptions alliance = allianceOptions.BLUE_ALLIANCE;
    private static startingPosition startingPos = startingPosition.FAR;

    private Bot(OpMode opMode) {
        this.opMode = opMode;


        drive = new MecanumDrive(opMode.hardwareMap, storedPose);
        turret = new Turret(opMode);
        intake = new Intake(opMode);
        lift = new Lift(opMode);
        updatePoses();
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
        turret.trackBlueAlliance();
        updatePoses();
    }

    public void setAllianceRed() {
        alliance = allianceOptions.RED_ALLIANCE;
        turret.trackRedAlliance();
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
            farAutoGoalPose = new Vector2d(farAutoGoalPose.x, -1 * Math.abs(farAutoGoalPose.y));
            resetPose = new Pose2d(resetPose.position.x, Math.abs(resetPose.position.y), Math.abs(resetPose.heading.log()));
        } else {
            goalPose = new Vector2d(goalPose.x, Math.abs(goalPose.y));
            farAutoGoalPose = new Vector2d(farAutoGoalPose.x, Math.abs(farAutoGoalPose.y));
            resetPose = new Pose2d(resetPose.position.x, -1 * Math.abs(resetPose.position.y), -1 * Math.abs(resetPose.heading.log()));
        }
        targetPose = goalPose;
    }

    public void setTargetFarAutoGoal() {
        targetPose = farAutoGoalPose;
    }

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

    public void enableFullAuto(boolean on) {
        turret.enableFullAuto(on);
    }

    public void enableShooter(boolean on) {
        turret.enableShooter(on);
    }

    public Action enableShooter() {
        return new InstantAction(()-> enableShooter(true));
    }

    public Action disableShooter() {
        return new InstantAction(() -> enableShooter(false));
    }

    public Action shootOne() {
        return new SequentialAction(
                new InstantAction(() -> shooting = true),
                new InstantAction(() -> intake.intake()),
                new SleepAction(0.1),
                new InstantAction(() -> intake.openGate()),
                new SleepAction(shootTime),
                new InstantAction(() -> intake.closeGate()),
                new InstantAction(() -> intake.storage()),
                new InstantAction(() -> shooting = false)
        );
    }

    public void updateShootingTime() {
        shootDelay = Math.max((Turret.shooterRpm - shootDelayRPMThreshold), 0) * shootDelayCF;
    }

    public Action shootThree() {
        updateShootingTime();
        return new SequentialAction(
                new InstantAction(() -> shooting = true),
                new InstantAction(() -> intake.intake()),
                new SleepAction(0.1),
                new InstantAction(() -> intake.openGate()),
                new SleepAction(shootTime),
                new InstantAction(() -> intake.closeGate()),
                new SleepAction(shootDelay),
                new InstantAction(() -> intake.openGate()),
                new SleepAction(shootTime),
                new InstantAction(() -> intake.closeGate()),
                new SleepAction(shootDelay),
                new InstantAction(() -> intake.openGate()),
                new SleepAction(shootTime),
                new InstantAction(() -> intake.closeGate()),
                new InstantAction(() -> intake.storage()),
                new InstantAction(() -> shooting = false)
        );
    }

    public Action shootThreeAutoFar() {
        updateShootingTime();
        setTargetFarAutoGoal();
        return new SequentialAction(
                new InstantAction(() -> shooting = true),
                new InstantAction(() -> intake.intake()),
                new SleepAction(0.1),
                new InstantAction(() -> intake.openGate()),
                new SleepAction(shootTime),
                new InstantAction(() -> intake.closeGate()),
                new SleepAction(autoFarShootDeley),
                new InstantAction(() -> intake.openGate()),
                new SleepAction(shootTime),
                new InstantAction(() -> intake.closeGate()),
                new SleepAction(autoFarShootDeley),
                new InstantAction(() -> intake.openGate()),
                new SleepAction(shootTime + 0.1),
                new InstantAction(() -> intake.closeGate()),
                new InstantAction(() -> intake.storage()),
                new InstantAction(this::setTargetGoalPose),
                new InstantAction(() -> shooting = false)
        );
    }

    public void periodic() {
        turret.periodic();
        lift.periodic();
    }

    public Action actionPeriodic() {
        return new actionPeriodic();
    }
    public class actionPeriodic implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            turret.periodic();
            lift.periodic();
            return true;
        }
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

//    public void driveRobotCentric(double strafeSpeed, double forwardBackSpeed, double turnSpeed) {
//        double frontWheelModifier = 1;
//        double rearWheelModifier = 1;
//        double[] speeds = {
//                (forwardBackSpeed + strafeSpeed + turnSpeed) * frontWheelModifier,
//                (forwardBackSpeed - strafeSpeed - turnSpeed) * frontWheelModifier,
//                (forwardBackSpeed - strafeSpeed + turnSpeed) * rearWheelModifier,
//                (forwardBackSpeed + strafeSpeed - turnSpeed) * rearWheelModifier
//        };
//        double maxSpeed = 0;
//        for (int i = 0; i < 4; i++) {
//            maxSpeed = Math.max(maxSpeed, speeds[i]);
//        }
//        if (maxSpeed > 1) {
//            for (int i = 0; i < 4; i++) {
//                speeds[i] /= maxSpeed;
//            }
//        }
//        fl.set(speeds[0]);
//        fr.set(-speeds[1]);
//        bl.set(speeds[2]);
//        br.set(-speeds[3]);
//    }


//    public void fixMotors() {
//        fl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//        fr.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//        bl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//        br.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//
////        fl.setInverted(false);
////        fr.setInverted(true);
////        bl.setInverted(false);
////        br.setInverted(true);
//
//        fl.setRunMode(Motor.RunMode.RawPower);
//        fr.setRunMode(Motor.RunMode.RawPower);
//        bl.setRunMode(Motor.RunMode.RawPower);
//        br.setRunMode(Motor.RunMode.RawPower);
//    }

//    public void stopMotors() {
//        fl.set(0.0);
//        fr.set(0.0);
//        bl.set(0.0);
//        br.set(0.0);
//    }

//    public double getMotorCurrent() {
//        return fl.motorEx.getCurrent(CurrentUnit.MILLIAMPS) + fr.motorEx.getCurrent(CurrentUnit.MILLIAMPS) + bl.motorEx.getCurrent(CurrentUnit.MILLIAMPS) + br.motorEx.getCurrent(CurrentUnit.MILLIAMPS);
//    }



}