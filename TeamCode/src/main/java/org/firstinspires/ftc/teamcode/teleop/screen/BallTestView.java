package org.firstinspires.ftc.teamcode.teleop.screen;

import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.DisplayView;

public class BallTestView extends DisplayView {

//    BallRegion ball1 = new BallRegion(0, 0);
//    BallRegion ball2 = new BallRegion(8, 0);
//    BallRegion ball3 = new BallRegion(16, 0);

    public BallTestView(BallRegion ball1) {
        super(new DisplayRegion[] {
                ball1
        });

    }
}
