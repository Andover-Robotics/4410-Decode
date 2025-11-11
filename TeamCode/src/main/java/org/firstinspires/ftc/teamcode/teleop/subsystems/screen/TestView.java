package org.firstinspires.ftc.teamcode.teleop.subsystems.screen;

import org.firstinspires.ftc.teamcode.teleop.subsystems.screen.FirstRegion;
import org.firstinspires.ftc.teamcode.teleop.subsystems.screen.SecondRegion;
import org.firstinspires.ftc.teamcode.teleop.subsystems.screen.ThirdRegion;

import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.DisplayView;

public class TestView extends DisplayView {
    public TestView() {
        super(new DisplayRegion[]{
                new FirstRegion(0,0, 8, 8),
                new SecondRegion(8,0,8,8),
                new ThirdRegion(16,0,8,8)
        });
    }
}
