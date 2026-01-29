package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "Lower Climb", group = "CompUtil")
public class LowerClimb extends LinearOpMode {

    private Bot bot;
    private double driveSpeed = 1, driveMultiplier = 1 ;
    private GamepadEx gp1, gp2;
    private List<Action> runningActions = new ArrayList<>();
    private boolean headingLockEnabled = true;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        Bot.instance = null;
        bot = Bot.getInstance(this);

        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);
        bot.enableFullAuto(false);

        // Initialize bot
//        bot.stopMotors();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            bot.clearBulkCache();
            TelemetryPacket packet = new TelemetryPacket();

            gp1.readButtons();
            gp2.readButtons();

            // CLIMB

            if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
                bot.lift.enableClosedLoop(!bot.lift.isClosedLoopEnabled());
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.X)) {
                bot.lift.lower();
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.TOUCHPAD)) {
                headingLockEnabled = !headingLockEnabled;
            }

            bot.lift.periodic();

            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }

            if (Math.abs(gp1.getLeftY()) > 0) {
                bot.lift.joystickLower(gp1.getLeftY());
            }
            runningActions = newActions;

            // TELEMETRY

            telemetry.addData("\nLeft Climb Position", bot.lift.getLeftEncContinuousDeg());
            telemetry.addData("Right Climb Position", bot.lift.getRightEncContinuousDeg());
            telemetry.addData("\nLeft Climb Abs Position", bot.lift.getLeftEncAbsDeg());
            telemetry.addData("Right Climb Abs Position", bot.lift.getRightEncAbsDeg());

            telemetry.addData("Climb Loop?", bot.lift.isClosedLoopEnabled());
            telemetry.addData("\nLeft Power", bot.lift.leftPower);
            telemetry.addData("Right Power", bot.lift.rightPower);
            telemetry.addData("\nActual Left Power", bot.lift.climbLeft.get());
            telemetry.addData("Actual Right Power", bot.lift.climbRight.get());
            telemetry.addData("\nLeft PID out", bot.lift.leftPidOut);
            telemetry.addData("Right PID out", bot.lift.rightPidOut);
            telemetry.addData("\nLeft Climb Target", bot.lift.leftTargetDeg);
            telemetry.addData("Right Climb Target", bot.lift.rightTargetDeg);
            telemetry.update();


        }
    }

    // Driving
    private void drive() { // Robot centric, drive multiplier default 1
        driveSpeed = driveMultiplier - 0.5 * gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        driveSpeed = Math.max(0, driveSpeed);
        if (headingLockEnabled) {
            bot.strafeHeadingLock(-gp1.getLeftX(), driveSpeed);
        } else {
            bot.driveRobotCentric(gp1.getLeftY(), -gp1.getLeftX(), -gp1.getRightX(), driveSpeed);
        }
    }
}
