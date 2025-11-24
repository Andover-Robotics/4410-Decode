package org.firstinspires.ftc.teamcode.teleop.subsystems.screen;

import com.acmerobotics.dashboard.canvas.Circle;

import team.techtigers.core.display.Color;
import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.sprites.Sprite;

public class CircleTestRegion extends DisplayRegion {
    private final Sprite[] sprites;

    public CircleTestRegion(int x, int y, int width, int height) {
        super(x,y,width,height);
        this.sprites = new Sprite[]{
//                new RectangleSprite(0,3,1, 4, Color.GREEN)
            new CircleSprite(4, 4, 4, Color.PURPLE)
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
