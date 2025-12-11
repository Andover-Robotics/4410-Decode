package org.firstinspires.ftc.teamcode.teleop.screen;

import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;

import team.techtigers.core.display.Color;
import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.sprites.RectangleSprite;
import team.techtigers.core.display.sprites.Sprite;
import team.techtigers.core.display.sprites.XSprite;

public class BallRegion extends DisplayRegion {
    private final CircleSprite circle1, circle2, circle3;
    private final CircleOutlineSprite outline1, outline2, outline3;
    private final Sprite[] sprites;
    public boolean teleop = false;
    public Bot bot;

    public BallRegion(int x, int y) {
        super(x, y, 24, 8);
        circle1 = new CircleSprite(0, 0, 8);
        circle1.setColor(Color.GREEN);
        circle1.enable();
        circle2 = new CircleSprite(8, 0, 8);
        circle2.setColor(Color.GREEN);
        circle2.enable();
        circle3 = new CircleSprite(16, 0, 8);
        circle3.setColor(Color.GREEN);
        circle3.enable();
        outline1 = new CircleOutlineSprite(0, 0, 8);
        outline1.setColor(Color.WHITE);
        outline1.disable();
        outline2 = new CircleOutlineSprite(8, 0, 8);
        outline2.setColor(Color.WHITE);
        outline2.disable();
        outline3 = new CircleOutlineSprite(16, 0, 8);
        outline3.setColor(Color.WHITE);
        outline3.disable();
        sprites = new Sprite[]{circle1, circle2, circle3, outline1, outline2, outline3};
    }

    @Override
    public void update() {
        if (teleop) {
            if (bot.intake.holdingTop()) {
                circle1.setColor(Color.GREEN);
            } else {
                circle1.setColor(Color.BLACK);
            }

            if (bot.intake.holdingMiddle()) {
                circle2.setColor(Color.GREEN);
            } else {
                circle2.setColor(Color.BLACK);
            }

            if (bot.intake.holdingBottom()) {
                circle3.setColor(Color.GREEN);
            } else {
                circle3.setColor(Color.BLACK);
            }

        } else if (ScreenTester.changed) {
            for (int i = 0; i < 3; i++) {
                Sprite circle = sprites[i];
                if (circle.getColor().equals(Color.GREEN)) {
                    circle.setColor(Color.BLACK);
                } else {
                    circle.setColor(Color.GREEN);
                }
            }
            ScreenTester.changed = false;
        } else {
            for (Sprite sprite : sprites) {
                sprite.setColor(sprite.getColor());
            }
        }
    }

    public void setTeleop(boolean t, Bot bot) {
        teleop = t;
        this.bot = bot;
    }

    @Override
    protected Sprite[] getSprites() {
        return this.sprites;
    }

}
