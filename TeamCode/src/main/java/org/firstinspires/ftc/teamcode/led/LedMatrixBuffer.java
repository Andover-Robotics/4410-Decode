package org.firstinspires.ftc.teamcode.led;

import java.util.Arrays;

/**
 * Frame buffer for an 8x24 monochrome matrix. Brightness values are stored as
 * bytes (0-255). Utility methods are provided to draw primitive shapes and
 * digits without needing to know the underlying pixel order.
 */
public class LedMatrixBuffer {
    public static final int ROWS = 8;
    public static final int COLS = 24;

    private final int[][] buffer = new int[ROWS][COLS];

    public void clear() {
        for (int r = 0; r < ROWS; r++) {
            Arrays.fill(buffer[r], 0);
        }
    }

    public void fill(int brightness) {
        int clamped = clamp(brightness);
        for (int r = 0; r < ROWS; r++) {
            Arrays.fill(buffer[r], clamped);
        }
    }

    public void setPixel(int row, int col, int brightness) {
        if (row < 0 || row >= ROWS || col < 0 || col >= COLS) {
            return;
        }
        buffer[row][col] = clamp(brightness);
    }

    public int getPixel(int row, int col) {
        if (row < 0 || row >= ROWS || col < 0 || col >= COLS) {
            return 0;
        }
        return buffer[row][col];
    }

    public void drawRectangle(int top, int left, int height, int width, int brightness, boolean filled) {
        int clamped = clamp(brightness);
        for (int r = top; r < top + height; r++) {
            for (int c = left; c < left + width; c++) {
                boolean onBorder = r == top || r == top + height - 1 || c == left || c == left + width - 1;
                if (filled || onBorder) {
                    setPixel(r, c, clamped);
                }
            }
        }
    }

    public void drawCircle(int centerRow, int centerCol, int radius, int brightness, boolean filled) {
        int clamped = clamp(brightness);
        int r2 = radius * radius;
        for (int y = -radius; y <= radius; y++) {
            for (int x = -radius; x <= radius; x++) {
                int dist2 = x * x + y * y;
                boolean onEdge = Math.abs(dist2 - r2) <= radius; // crude edge approximation
                if (dist2 <= r2 && (filled || onEdge)) {
                    setPixel(centerRow + y, centerCol + x, clamped);
                }
            }
        }
    }

    public void drawDigit(int top, int left, int digit, int brightness) {
        if (digit < 0 || digit > 9) {
            return;
        }
        int clamped = clamp(brightness);
        boolean[][] pattern = DIGITS[digit];
        for (int r = 0; r < pattern.length; r++) {
            for (int c = 0; c < pattern[r].length; c++) {
                if (pattern[r][c]) {
                    setPixel(top + r, left + c, clamped);
                }
            }
        }
    }

    public byte[] toNeoPixelBuffer() {
        byte[] pixels = new byte[ROWS * COLS * 3];
        int index = 0;
        // serpentine wiring: even rows left->right, odd rows right->left
        for (int row = 0; row < ROWS; row++) {
            boolean reverse = row % 2 == 1;
            for (int col = 0; col < COLS; col++) {
                int sourceCol = reverse ? (COLS - 1 - col) : col;
                int value = buffer[row][sourceCol];
                byte b = (byte) value;
                // GRB order for NeoPixels
                pixels[index++] = b; // G
                pixels[index++] = b; // R
                pixels[index++] = b; // B
            }
        }
        return pixels;
    }

    private int clamp(int brightness) {
        return Math.max(0, Math.min(255, brightness));
    }

    // Simple 3x5 font for digits 0-9
    private static final boolean[][][] DIGITS = new boolean[][][]{
            {
                    {true, true, true},
                    {true, false, true},
                    {true, false, true},
                    {true, false, true},
                    {true, true, true}
            },
            {
                    {false, true, false},
                    {true, true, false},
                    {false, true, false},
                    {false, true, false},
                    {true, true, true}
            },
            {
                    {true, true, true},
                    {false, false, true},
                    {true, true, true},
                    {true, false, false},
                    {true, true, true}
            },
            {
                    {true, true, true},
                    {false, false, true},
                    {true, true, true},
                    {false, false, true},
                    {true, true, true}
            },
            {
                    {true, false, true},
                    {true, false, true},
                    {true, true, true},
                    {false, false, true},
                    {false, false, true}
            },
            {
                    {true, true, true},
                    {true, false, false},
                    {true, true, true},
                    {false, false, true},
                    {true, true, true}
            },
            {
                    {true, true, true},
                    {true, false, false},
                    {true, true, true},
                    {true, false, true},
                    {true, true, true}
            },
            {
                    {true, true, true},
                    {false, false, true},
                    {false, true, false},
                    {false, true, false},
                    {false, true, false}
            },
            {
                    {true, true, true},
                    {true, false, true},
                    {true, true, true},
                    {true, false, true},
                    {true, true, true}
            },
            {
                    {true, true, true},
                    {true, false, true},
                    {true, true, true},
                    {false, false, true},
                    {true, true, true}
            }
    };
}
