package org.firstinspires.ftc.teamcode.Prism;

import java.util.EnumMap;
import java.util.Map;

/**
 * Pre-builds a BallScreen "animation" using four logical layers (three balls and a background)
 * and three artboards per layer (empty, green, purple for the balls; transparent, red, yellow for
 * the background). Each artboard renders only its own layer content so that the layers can be
 * composed in software and streamed directly to the Prism via {@link GoBildaPrismDriver#displayFrame(Color[])}.
 */
public class BallScreenAnimator {

    /** States for each ball artboard. */
    public enum BallArtboard {
        NOTHING,
        GREEN,
        PURPLE
    }

    /** States for the background artboard. */
    public enum BackgroundArtboard {
        NOTHING(Color.TRANSPARENT),
        RED(Color.RED),
        YELLOW(Color.YELLOW);

        private final Color color;

        BackgroundArtboard(Color color) {
            this.color = color;
        }

        public Color getColor() {
            return color;
        }
    }

    /** Logical layers (artboard sets) for the animation. */
    public enum Layer {
        BACKGROUND,
        BALL_0,
        BALL_1,
        BALL_2
    }

    private final Color[][] backgroundArtboards = new Color[BackgroundArtboard.values().length][];
    private final Color[][][] ballArtboards = new Color[3][BallArtboard.values().length][];

    /**
     * Build all layer artboards up front so they can be recalled quickly.
     */
    public BallScreenAnimator() {
        buildBackgrounds();
        buildBallArtboards();
    }

    private void buildBackgrounds() {
        for (BackgroundArtboard artboard : BackgroundArtboard.values()) {
            Color[][] frame = new Color[GoBildaPrismDriver.MATRIX_HEIGHT][GoBildaPrismDriver.MATRIX_WIDTH];
            for (int y = 0; y < GoBildaPrismDriver.MATRIX_HEIGHT; y++) {
                for (int x = 0; x < GoBildaPrismDriver.MATRIX_WIDTH; x++) {
                    frame[y][x] = artboard.getColor();
                }
            }
            backgroundArtboards[artboard.ordinal()] = flatten(frame);
        }
    }

    private void buildBallArtboards() {
        int[] centersX = new int[]{4, 12, 20};
        int centerY = 3;

        for (int ball = 0; ball < 3; ball++) {
            for (BallArtboard artboard : BallArtboard.values()) {
                Color[][] frame = new Color[GoBildaPrismDriver.MATRIX_HEIGHT][GoBildaPrismDriver.MATRIX_WIDTH];
                // Transparent base so layers can be composed on top of the background.
                for (int y = 0; y < GoBildaPrismDriver.MATRIX_HEIGHT; y++) {
                    for (int x = 0; x < GoBildaPrismDriver.MATRIX_WIDTH; x++) {
                        frame[y][x] = Color.TRANSPARENT;
                    }
                }

                if (artboard != BallArtboard.NOTHING) {
                    paintBall(frame, centersX[ball], centerY, artboard == BallArtboard.GREEN ? Color.GREEN : Color.PURPLE);
                }

                ballArtboards[ball][artboard.ordinal()] = flatten(frame);
            }
        }
    }

    /** Compose the currently selected artboards into a single LED buffer. */
    public Color[] compose(Map<Layer, Enum<?>> selection) {
        Color[][] frame = new Color[GoBildaPrismDriver.MATRIX_HEIGHT][GoBildaPrismDriver.MATRIX_WIDTH];

        // Start with the background
        BackgroundArtboard background = (BackgroundArtboard) selection.getOrDefault(Layer.BACKGROUND, BackgroundArtboard.NOTHING);
        inflate(backgroundArtboards[background.ordinal()], frame);

        // Overlay balls in order so later layers sit on top of earlier ones.
        for (int layer = 0; layer < 3; layer++) {
            BallArtboard artboard = (BallArtboard) selection.getOrDefault(Layer.values()[layer + 1], BallArtboard.NOTHING);
            overlay(ballArtboards[layer][artboard.ordinal()], frame);
        }

        return flatten(frame);
    }

    private void overlay(Color[] layer, Color[][] destination) {
        for (int y = 0; y < GoBildaPrismDriver.MATRIX_HEIGHT; y++) {
            for (int x = 0; x < GoBildaPrismDriver.MATRIX_WIDTH; x++) {
                Color color = layer[y * GoBildaPrismDriver.MATRIX_WIDTH + x];
                if (color != Color.TRANSPARENT) {
                    destination[y][x] = color;
                }
            }
        }
    }

    private void inflate(Color[] flat, Color[][] dest) {
        for (int y = 0; y < GoBildaPrismDriver.MATRIX_HEIGHT; y++) {
            for (int x = 0; x < GoBildaPrismDriver.MATRIX_WIDTH; x++) {
                dest[y][x] = flat[y * GoBildaPrismDriver.MATRIX_WIDTH + x];
            }
        }
    }

    private Color[] flatten(Color[][] frame) {
        Color[] flat = new Color[GoBildaPrismDriver.MATRIX_WIDTH * GoBildaPrismDriver.MATRIX_HEIGHT];
        for (int y = 0; y < GoBildaPrismDriver.MATRIX_HEIGHT; y++) {
            for (int x = 0; x < GoBildaPrismDriver.MATRIX_WIDTH; x++) {
                flat[y * GoBildaPrismDriver.MATRIX_WIDTH + x] = frame[y][x];
            }
        }
        return flat;
    }

    /**
     * Returns the prebuilt artboard frame for a given layer and artboard selection.
     */
    public Color[] getArtboardFrame(Layer layer, Enum<?> artboard) {
        switch (layer) {
            case BACKGROUND:
                BackgroundArtboard bg = (BackgroundArtboard) artboard;
                return backgroundArtboards[bg.ordinal()];
            case BALL_0:
            case BALL_1:
            case BALL_2:
                int ballIndex = layer.ordinal() - 1;
                BallArtboard ballArtboard = (BallArtboard) artboard;
                return ballArtboards[ballIndex][ballArtboard.ordinal()];
            default:
                throw new IllegalArgumentException("Unknown layer: " + layer);
        }
    }

    private void paintBall(Color[][] frame, int centerX, int centerY, Color fillColor) {
        for (int y = 0; y < GoBildaPrismDriver.MATRIX_HEIGHT; y++) {
            for (int x = 0; x < GoBildaPrismDriver.MATRIX_WIDTH; x++) {
                double distance = Math.hypot(x - centerX, y - centerY);
                if (distance <= 2) {
                    frame[y][x] = fillColor;
                } else if (distance <= 3) {
                    frame[y][x] = Color.WHITE;
                }
            }
        }
    }

    /** Utility for building a default selection map. */
    public Map<Layer, Enum<?>> defaultSelection() {
        Map<Layer, Enum<?>> selection = new EnumMap<>(Layer.class);
        selection.put(Layer.BACKGROUND, BackgroundArtboard.NOTHING);
        selection.put(Layer.BALL_0, BallArtboard.NOTHING);
        selection.put(Layer.BALL_1, BallArtboard.NOTHING);
        selection.put(Layer.BALL_2, BallArtboard.NOTHING);
        return selection;
    }
}
