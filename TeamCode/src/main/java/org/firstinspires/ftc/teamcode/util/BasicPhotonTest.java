//package org.firstinspires.ftc.teamcode.util;
//
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.seattlesolvers.solverslib.photon.PhotonCore;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.seattlesolvers.solverslib.geometry.Pose2d;
//import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
//import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
//@Config
//@TeleOp(group = "Photon")
//public class BasicPhotonTest extends LinearOpMode {
//    private ElapsedTime timer = new ElapsedTime();
//    public static boolean ENABLE_PHOTON = true;
//    public static boolean MOTOR = true;
//    public static boolean SWERVO = true;
//    public static double INTAKE_MOTOR_CACHE_TOL = -0.01;
//    public static double SWERVO_CACHE_TOL = -0.01;
//    public static double WRITES = 1;
//    public static double READS = 1;
//    public static double PINPOINT_READS = 1;
//    public static double POWER_INCREMENT = 0.0;
//    double power = 0;
//
//    @Override
//    public void runOpMode() {
//
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.experimental.setMaximumParallelCommands(8);
//        PhotonCore.experimental.setSinglethreadedOptimized(false);
//
//        if (ENABLE_PHOTON) {
//            PhotonCore.enable();
//        }
//
//        MotorEx intakeMotor = new MotorEx(hardwareMap, "intake").setCachingTolerance(INTAKE_MOTOR_CACHE_TOL);
//        CRServoEx swervo = new CRServoEx(hardwareMap, "climbLeft").setCachingTolerance(SWERVO_CACHE_TOL);
//        SolversPinpointDriver pinpoint = hardwareMap.get(SolversPinpointDriver.class, "pinpoint").setOffsets(-76.32, 152.62, DistanceUnit.MM)
//                .setEncoderResolution(SolversPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
//                .setEncoderDirections(SolversPinpointDriver.EncoderDirection.REVERSED, SolversPinpointDriver.EncoderDirection.REVERSED)
//                .setErrorDetectionType(SolversPinpointDriver.ErrorDetectionType.CRC)
//                .resetPosAndIMU()
//                .setBulkReadScope(SolversPinpointDriver.Register.X_POSITION, SolversPinpointDriver.Register.Y_POSITION, SolversPinpointDriver.Register.H_ORIENTATION, SolversPinpointDriver.Register.X_VELOCITY, SolversPinpointDriver.Register.Y_VELOCITY, SolversPinpointDriver.Register.H_VELOCITY);
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            timer.reset();
//            intakeMotor.setCachingTolerance(INTAKE_MOTOR_CACHE_TOL);
//            swervo.setCachingTolerance(SWERVO_CACHE_TOL);
//
//            for (int i = 0; i < WRITES; i++) {
//                power = power + POWER_INCREMENT;
//                if (power > 1) {
//                    power = -1;
//                }
//
//                if (MOTOR) {
//                    intakeMotor.set(power);
//                }
//                if (SWERVO) {
//                    swervo.set(power);
//                }
//            }
//
//            for (int i = 0; i < READS; i++) {
//                intakeMotor.encoder.getCorrectedVelocity();
//            }
//
//            for (int i = 0; i < PINPOINT_READS; i++) {
//                pinpoint.update();
//            }
//
//            PhotonCore.CONTROL_HUB.clearBulkCache();
//            PhotonCore.EXPANSION_HUB.clearBulkCache();
//
//            telemetry.addData("Loop Time (ms)", timer.milliseconds());
//            telemetry.update();
//        }
//    }
//}