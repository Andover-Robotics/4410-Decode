package org.firstinspires.ftc.teamcode.teleop.subsystems.screen;

import team.techtigers.core.display.Color;
import team.techtigers.core.display.sprites.Sprite;

public class CircleSprite extends Sprite {
    private int radius;

    public CircleSprite(int centerX, int centerY, int radius, Color color) {
        // Define bounds around the circle
        super(centerX - radius, centerY - radius, radius * 2, radius * 2);
        this.radius = radius;
        this.setColor(color);
    }

    @Override
    protected void showSprite(Color[][] leds) {
        // Use fractional center for symmetry on even grids
        double centerX = this.getX() + radius - 0.5;
        double centerY = this.getY() + radius - 0.5;

        int maxY = leds.length;
        int maxX = maxY > 0 ? leds[0].length : 0;

        for (int x = this.getX(); x < this.getX() + this.getWidth(); x++) {
            for (int y = this.getY(); y < this.getY() + this.getHeight(); y++) {
                if (y < 0 || y >= maxY || x < 0 || x >= maxX) {
                    continue;
                }

                double dx = x - centerX;
                double dy = y - centerY;

                if (dx * dx + dy * dy <= radius * radius) {
                    leds[y][x] = this.getColor();
                }
            }
        }
    }
}
