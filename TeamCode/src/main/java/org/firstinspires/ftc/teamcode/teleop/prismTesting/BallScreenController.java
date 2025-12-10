package org.firstinspires.ftc.teamcode.teleop.prismTesting;

import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.MATRIX_HEIGHT;
import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.MATRIX_WIDTH;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Prism.BallScreen;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;

/**
 * Demonstrates updating the BallScreen helper with gamepad input and sending the rendered frame
 * to the Prism. D-pad left/up/right cycle each ball between empty, green, and purple, while
 * d-pad down cycles the background between transparent, red, and yellow.
 */
@TeleOp(name = "Prism Ball Screen Controller", group = "Linear OpMode")
public class BallScreenController extends LinearOpMode {

    private final BallScreen ballScreen = new BallScreen();

    private final BallScreen.BallState[] states = new BallScreen.BallState[]{
            BallScreen.BallState.EMPTY,
            BallScreen.BallState.EMPTY,
            BallScreen.BallState.EMPTY
    };

    private final Color[] backgroundCycle = new Color[]{Color.TRANSPARENT, Color.RED, Color.YELLOW};
    private int backgroundIndex = 0;

    private GamepadEx gp1;

    private GoBildaPrismDriver prism;

    @Override
    public void runOpMode() {
        gp1 = new GamepadEx(gamepad1);
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        prism.setStripLength(GoBildaPrismDriver.MATRIX_LED_COUNT);

        ballScreen.setBackgroundColor(backgroundCycle[backgroundIndex]);
        updateFrame();

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            gp1.readButtons();
            boolean updated = false;

            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                updated |= cycleBall(0);
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                updated |= cycleBall(1);
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                updated |= cycleBall(2);
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                backgroundIndex = (backgroundIndex + 1) % backgroundCycle.length;
                ballScreen.setBackgroundColor(backgroundCycle[backgroundIndex]);
                updated = true;
            }

            if (updated) {
                updateFrame();
            }

            telemetry.addLine("D-Pad: left/up/right change balls, down changes background");
            telemetry.addData("Ball 1", states[0]);
            telemetry.addData("Ball 2", states[1]);
            telemetry.addData("Ball 3", states[2]);
            telemetry.addData("Background", describeBackground());
            telemetry.addData("Frame Size", MATRIX_WIDTH + "x" + MATRIX_HEIGHT);
            telemetry.update();

            sleep(50);
        }
    }

    private boolean cycleBall(int index) {
        BallScreen.BallState next = nextState(states[index]);
        states[index] = next;
        ballScreen.setBallState(index, next);
        return true;
    }

    private BallScreen.BallState nextState(BallScreen.BallState current) {
        switch (current) {
            case EMPTY:
                return BallScreen.BallState.GREEN;
            case GREEN:
                return BallScreen.BallState.PURPLE;
            default:
                return BallScreen.BallState.EMPTY;
        }
    }

    private String describeBackground() {
        Color color = backgroundCycle[backgroundIndex];
        if (color == Color.TRANSPARENT) {
            return "Transparent";
        }
        if (color == Color.RED) {
            return "Red";
        }
        return "Yellow";
    }

    private void updateFrame() {
        prism.displayFrame(ballScreen.renderAsLedStrip());
    }
}
