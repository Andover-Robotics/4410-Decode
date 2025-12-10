package org.firstinspires.ftc.teamcode.Prism;

import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.MATRIX_HEIGHT;
import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.MATRIX_WIDTH;
import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.matrixIndexFromCoordinate;

/**
 * Builds a simple 24 x 8 (width x height) pixel buffer that represents three balls on the Prism panel.
 * <p>
 * Each ball always renders with a single-pixel white outline. The fill color can be changed to purple,
 * green, or cleared back to the background color. The background color of the whole frame can also be
 * updated.
 * </p>
 */
public class BallScreen {
    /** Represents what the inside of the ball should display. */
    public enum BallState {
        EMPTY,
        GREEN,
        PURPLE
    }

    private static final int DEFAULT_OUTLINE_RADIUS = 3;
    private static final int DEFAULT_FILL_RADIUS = 2;
    private static final int BALL_COUNT = 3;

    private final BallState[] ballStates = new BallState[]{BallState.EMPTY, BallState.EMPTY, BallState.EMPTY};
    private final int[] centerXs = new int[]{4, 12, 20};
    private final int centerY = 3;

    private Color backgroundColor = Color.TRANSPARENT;
    private Color outlineColor = Color.WHITE;

    /** Set the fill color for a specific ball index (0 based). */
    public void setBallState(int index, BallState state) {
        if (index < 0 || index >= BALL_COUNT) {
            throw new IllegalArgumentException("Ball index must be between 0 and 2 inclusive.");
        }
        ballStates[index] = state;
    }

    /** Change the background color for every pixel not used by the balls. */
    public void setBackgroundColor(Color backgroundColor) {
        if (backgroundColor == null) {
            throw new IllegalArgumentException("Background color cannot be null.");
        }
        this.backgroundColor = backgroundColor;
    }

    /** Update the outline color around every ball. */
    public void setOutlineColor(Color outlineColor) {
        if (outlineColor == null) {
            throw new IllegalArgumentException("Outline color cannot be null.");
        }
        this.outlineColor = outlineColor;
    }

    /**
     * Render the current configuration as a two-dimensional color array using cartesian coordinates.
     * The first index is Y (row), and the second is X (column).
     */
    public Color[][] renderFrame() {
        Color[][] frame = new Color[MATRIX_HEIGHT][MATRIX_WIDTH];
        for (int y = 0; y < MATRIX_HEIGHT; y++) {
            for (int x = 0; x < MATRIX_WIDTH; x++) {
                frame[y][x] = backgroundColor;
            }
        }

        for (int i = 0; i < BALL_COUNT; i++) {
            paintBall(frame, centerXs[i], centerY, ballStates[i]);
        }

        return frame;
    }

    /**
     * Flatten the 2D frame into the serpentine LED order used by the Prism driver.
     */
    public Color[] renderAsLedStrip() {
        Color[][] frame = renderFrame();
        Color[] strip = new Color[MATRIX_WIDTH * MATRIX_HEIGHT];

        for (int y = 0; y < MATRIX_HEIGHT; y++) {
            for (int x = 0; x < MATRIX_WIDTH; x++) {
                int index = matrixIndexFromCoordinate(x, y);
                strip[index] = frame[y][x];
            }
        }

        return strip;
    }

    private void paintBall(Color[][] frame, int centerX, int centerY, BallState state) {
        Color fillColor = backgroundColor;
        if (state == BallState.GREEN) {
            fillColor = Color.GREEN;
        } else if (state == BallState.PURPLE) {
            fillColor = Color.PURPLE;
        }

        for (int y = 0; y < MATRIX_HEIGHT; y++) {
            for (int x = 0; x < MATRIX_WIDTH; x++) {
                double distance = Math.hypot(x - centerX, y - centerY);
                if (distance <= DEFAULT_FILL_RADIUS) {
                    frame[y][x] = fillColor;
                } else if (distance <= DEFAULT_OUTLINE_RADIUS) {
                    frame[y][x] = outlineColor;
                }
            }
        }
    }
}
