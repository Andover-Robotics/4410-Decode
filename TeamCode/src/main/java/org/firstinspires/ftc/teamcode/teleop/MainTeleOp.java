package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;

import java.lang.*;
import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends LinearOpMode {

    private Bot bot;
    private double driveSpeed = 1, driveMultiplier = 1;
    private GamepadEx gp1, gp2;
    private boolean fieldCentric, intakeCancel, clipCancel;
    private Thread thread;
    private List<Action> runningActions = new ArrayList<>();

    private int degTarget = 0;
    private boolean runTurret = false;
    public static double rpm = 0;
    public static boolean shoot = false;
    public static boolean noPid = false;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Bot.instance = null;
        bot = Bot.getInstance(this);

        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);

        // Initialize bot
//        bot.stopMotors();

        //bot.storage();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();

            gp1.readButtons();
            gp2.readButtons();

            if (gp1.wasJustPressed(GamepadKeys.Button.START)) {
                bot.resetHeading();
            }

            telemetry.addData("Heading (deg)", bot.getHeading());

            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                runTurret = false;
                bot.turret.enableAutoAim(false); //disable auto aim

            }
            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                runTurret = true;
            }
            // enable/disable auto-aim with LEFT_BUMPER
            if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                bot.turret.enableAutoAim(true);
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                bot.turret.enableAutoAim(false);
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.X) || noPid) {
                bot.shooter.setManualPower(rpm);
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.Y) || !noPid) {
                bot.shooter.setVelocity(rpm);
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                degTarget += 20;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                degTarget -= 20;
            }



            // DRIVE
//            drive();

//            // TELEMETRY
//            telemetry.addData("On?", runTurret);
//            telemetry.addData("Temp Target (Degs)", degTarget);
//
//            telemetry.addData("\nTarget (Ticks)", bot.turret.getTargetTicks());
//            telemetry.addData("Target (Degs)", bot.turret.getTargetDegs());
//            telemetry.addData("Pos (Ticks)", bot.turret.getPositionTicks());
//            telemetry.addData("Pos (Degs)", bot.turret.getPositionDegs());
//            telemetry.addData("Power", bot.turret.getPower());
            telemetry.addData("Turret AutoAim", bot.turret.getTargetDegs());
//            telemetry.addData("Power", bot.shooter.getPower());
//            telemetry.addData("measured rpm", bot.shooter.getMeasuredRPM());
//            telemetry.addData("filtered rpm", bot.shooter.getFilteredRPM());
//            telemetry.addData("target (power) rpm", rpm);
//            telemetry.addData("target (PIDF) rpm", bot.shooter.getTargetRPM());
//            telemetry.addData("On?", shoot);
            telemetry.addData("ty", bot.turret.ty);
            telemetry.addData("PID setPoint", bot.turret.setPoint);
            telemetry.addData("PID pos", bot.turret.pos);

            telemetry.update();

            bot.turret.runToAngle(degTarget);
            if (runTurret){
                bot.turret.periodic();
            }
            if (gp1.isDown(GamepadKeys.Button.RIGHT_BUMPER) || shoot) {
                bot.shooter.periodic();
            } else{
                bot.shooter.setPower(0);
            }
        }
    }

    // Driving
//    private void drive() { // Robot centric, drive multiplier default 1
//        driveSpeed = driveMultiplier - 0.5 * gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
//        driveSpeed = Math.max(0, driveSpeed);
//        bot.fixMotors();
//
//        if (fieldCentric) {
//            Vector2d driveVector = new Vector2d(-gp1.getLeftX(), -gp1.getLeftY()),
//                    turnVector = new Vector2d(-gp1.getRightX(), 0);
//            bot.driveFieldCentric(driveVector.getX() * driveSpeed,
//                    driveVector.getY() * driveSpeed,
//                    turnVector.getX() * driveSpeed
//            );
//        } else {
//            Vector2d driveVector = new Vector2d(gp1.getLeftX(), -gp1.getLeftY()),
//                    turnVector = new Vector2d(gp1.getRightX(), 0);
//            bot.driveRobotCentric(driveVector.getX() * driveSpeed,
//                    driveVector.getY() * driveSpeed,
//                    turnVector.getX() * driveSpeed
//            );
//        }
//    }

}