package org.firstinspires.ftc.teamcode.teleop.subsystems.screen;

import team.techtigers.core.display.Color;
import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.sprites.Sprite;

public class GreenSquareFirstRegion extends DisplayRegion {
    private final Sprite[] sprites;

    public GreenSquareFirstRegion(int x, int y, int width, int height) {
        super(x,y,width,height);
        this.sprites = new Sprite[]{
                new RectangleSprite(1,1,6, 6, Color.GREEN)
        };
    }

    @Override
    public void update() {

    }

    @Override
    protected Sprite[] getSprites() {
        return this.sprites;
    }
}
