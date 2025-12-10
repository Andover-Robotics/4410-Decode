package org.firstinspires.ftc.teamcode.teleop.prismTesting;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Prism.BallScreenAnimator;
import org.firstinspires.ftc.teamcode.Prism.BallScreenLayerAnimation;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.Artboard;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;

/**
 * Demonstrates building a "ballscreen" animation using four layers (three balls + background)
 * and saving/restoring artboards on the Prism. D-pad left/up/right cycle each ball between empty,
 * green, and purple, while d-pad down cycles the background between transparent, red, and yellow.
 * Left/right bumper save and load the current composition to/from artboard slots.
 */
@TeleOp(name = "Prism Ball Screen Controller", group = "Linear OpMode")
public class BallScreenController extends LinearOpMode {

    private final BallScreenAnimator animator = new BallScreenAnimator();
    private final java.util.Map<BallScreenAnimator.Layer, Enum<?>> selection = new java.util.EnumMap<>(BallScreenAnimator.Layer.class);
    private final java.util.Map<BallScreenAnimator.Layer, BallScreenLayerAnimation> layerAnimations = new java.util.EnumMap<>(BallScreenAnimator.Layer.class);
    private final java.util.Map<BallScreenAnimator.Layer, LayerHeight> layerHeights = new java.util.EnumMap<>(BallScreenAnimator.Layer.class);
    private int saveSlot = 0;

    private GamepadEx gp1;

    private GoBildaPrismDriver prism;

    @Override
    public void runOpMode() {
        gp1 = new GamepadEx(gamepad1);
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        prism.setStripLength(GoBildaPrismDriver.MATRIX_LED_COUNT);

        selection.putAll(animator.defaultSelection());

        layerHeights.put(BallScreenAnimator.Layer.BACKGROUND, LayerHeight.LAYER_0);
        layerHeights.put(BallScreenAnimator.Layer.BALL_0, LayerHeight.LAYER_1);
        layerHeights.put(BallScreenAnimator.Layer.BALL_1, LayerHeight.LAYER_2);
        layerHeights.put(BallScreenAnimator.Layer.BALL_2, LayerHeight.LAYER_3);

        // Build all four layers onto the Prism using the animation pipeline.
        for (BallScreenAnimator.Layer layer : BallScreenAnimator.Layer.values()) {
            pushLayerToPrism(layer);
        }

        // Save the initial composition to Artboard 0 so there is always a recall point.
        saveCurrentToArtboard(Artboard.ARTBOARD_0);

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            gp1.readButtons();
            boolean updated = false;

            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                updated |= cycleBall(BallScreenAnimator.Layer.BALL_0);
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                updated |= cycleBall(BallScreenAnimator.Layer.BALL_1);
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                updated |= cycleBall(BallScreenAnimator.Layer.BALL_2);
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                cycleBackground();
                updated = true;
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                saveCurrentToArtboard(Artboard.values()[saveSlot]);
                saveSlot = (saveSlot + 1) % Artboard.values().length;
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                loadArtboard(Artboard.values()[saveSlot]);
                saveSlot = (saveSlot + 1) % Artboard.values().length;
            }

            if (updated) {
                // Only resend the changed layers because each sits in its own height on the Prism.
                for (BallScreenAnimator.Layer layer : BallScreenAnimator.Layer.values()) {
                    if (layer == BallScreenAnimator.Layer.BACKGROUND && gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                        pushLayerToPrism(layer);
                    } else if (layer == BallScreenAnimator.Layer.BALL_0 && gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                        pushLayerToPrism(layer);
                    } else if (layer == BallScreenAnimator.Layer.BALL_1 && gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                        pushLayerToPrism(layer);
                    } else if (layer == BallScreenAnimator.Layer.BALL_2 && gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                        pushLayerToPrism(layer);
                    }
                }
            }

            telemetry.addLine("D-Pad: left/up/right change balls, down changes background");
            telemetry.addLine("LB: save current composition to next artboard slot");
            telemetry.addLine("RB: recall the next artboard slot");
            telemetry.addData("Ball 1", selection.get(BallScreenAnimator.Layer.BALL_0));
            telemetry.addData("Ball 2", selection.get(BallScreenAnimator.Layer.BALL_1));
            telemetry.addData("Ball 3", selection.get(BallScreenAnimator.Layer.BALL_2));
            telemetry.addData("Background", selection.get(BallScreenAnimator.Layer.BACKGROUND));
            telemetry.addData("Save/Load Slot", saveSlot);
            telemetry.update();

            sleep(50);
        }
    }

    private boolean cycleBall(BallScreenAnimator.Layer layer) {
        BallScreenAnimator.BallArtboard current = (BallScreenAnimator.BallArtboard) selection.get(layer);
        BallScreenAnimator.BallArtboard next = nextBallArtboard(current);
        selection.put(layer, next);
        return true;
    }

    private BallScreenAnimator.BallArtboard nextBallArtboard(BallScreenAnimator.BallArtboard current) {
        switch (current) {
            case NOTHING:
                return BallScreenAnimator.BallArtboard.GREEN;
            case GREEN:
                return BallScreenAnimator.BallArtboard.PURPLE;
            default:
                return BallScreenAnimator.BallArtboard.NOTHING;
        }
    }

    private void cycleBackground() {
        BallScreenAnimator.BackgroundArtboard current = (BallScreenAnimator.BackgroundArtboard) selection.get(BallScreenAnimator.Layer.BACKGROUND);
        BallScreenAnimator.BackgroundArtboard[] values = BallScreenAnimator.BackgroundArtboard.values();
        int nextIndex = (current.ordinal() + 1) % values.length;
        selection.put(BallScreenAnimator.Layer.BACKGROUND, values[nextIndex]);
    }

    private void pushLayerToPrism(BallScreenAnimator.Layer layer) {
        BallScreenLayerAnimation animation = layerAnimations.computeIfAbsent(layer, l -> new BallScreenLayerAnimation());
        animation.setFrame(animator.getArtboardFrame(layer, selection.get(layer)));

        LayerHeight targetHeight = layerHeights.get(layer);
        prism.insertAndUpdateAnimation(targetHeight, animation);
    }

    private void saveCurrentToArtboard(Artboard artboard) {
        // Persist the Prism-side animations for every layer into the requested artboard.
        prism.saveCurrentAnimationsToArtboard(artboard);
    }

    private void loadArtboard(Artboard artboard) {
        // Recall the saved animations directly from the Prism's artboard storage.
        prism.loadAnimationsFromArtboard(artboard);
    }
}
