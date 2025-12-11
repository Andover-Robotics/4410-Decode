package org.firstinspires.ftc.teamcode.teleop.screen;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import team.techtigers.core.display.AdafruitNeoPixel;
import team.techtigers.core.display.VisualDisplay;

@TeleOp(name = "Screen Testing OpMode", group = "Test")
public class ScreenTester extends LinearOpMode {

    BallRegion ball1 = new BallRegion(0, 0);
    private GamepadEx gp1, gp2;
    public static boolean changed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        AdafruitNeoPixel driver = hardwareMap.get(AdafruitNeoPixel.class, "screen");
        driver.initialize(192, 3);
        BallTestView ballTestView = new BallTestView(ball1);
        VisualDisplay screen = new VisualDisplay(driver, ballTestView);
        gp1 = new GamepadEx(gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            gp1.readButtons();
            screen.update();
            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                changed = true;
            }

        }
    }
}
