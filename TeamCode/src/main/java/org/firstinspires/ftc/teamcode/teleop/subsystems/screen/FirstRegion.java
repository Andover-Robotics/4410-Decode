package org.firstinspires.ftc.teamcode.teleop.subsystems.screen;

import team.techtigers.core.display.Color;
import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.sprites.Sprite;
import team.techtigers.core.display.sprites.numbers.OneSprite;
import team.techtigers.core.display.sprites.numbers.TwoSprite;
import team.techtigers.core.display.sprites.numbers.ZeroSprite;

public class FirstRegion extends DisplayRegion {
    private final Sprite[] sprites;
    private int frameCounter = 0;
    private boolean isRed = true;

    private final OneSprite oneSprite;

    private final TwoSprite twoSprite;

    public FirstRegion(int x, int y, int width, int height) {
        super(x,y,width,height);
        oneSprite = new OneSprite(1,1);
        oneSprite.setColor(Color.RED);
        twoSprite = new TwoSprite(1,1);
        twoSprite.setColor(Color.RED);
        this.sprites = new Sprite[]{
                new RectangleSprite(1,1,6, 6, Color.GREEN),
                oneSprite
        };
    }

    @Override
    public void update() {
        frameCounter = frameCounter + 1;
        if(frameCounter < 20) return;
        frameCounter = 0;

        if (isRed) {
            isRed = false;
            this.sprites[0].setColor(Color.GREEN);
            this.sprites[0] = twoSprite;
        } else {
            isRed = true;
            this.sprites[0].setColor(Color.RED);
            this.sprites[0] = oneSprite;
        }
    }

    @Override
    protected Sprite[] getSprites() {
        return this.sprites;
    }
}
