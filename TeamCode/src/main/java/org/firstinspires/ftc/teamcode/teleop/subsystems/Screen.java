package org.firstinspires.ftc.teamcode.teleop.subsystems;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import team.techtigers.core.display.AdafruitNeoPixel;
import team.techtigers.core.display.VisualDisplay;
import org.firstinspires.ftc.teamcode.teleop.*;
import org.firstinspires.ftc.teamcode.teleop.screen.*;

public class Screen {

    BallRegion balls = new BallRegion(0, 0);
    public VisualDisplay display;
    public BallView ballView;

    public Screen (OpMode opMode, Bot bot) {
        AdafruitNeoPixel driver = opMode.hardwareMap.get(AdafruitNeoPixel.class, "screen");
        driver.initialize(192, 3);
        balls.setTeleop(true, bot);
        ballView = new BallView(balls);
        display = new VisualDisplay(driver, ballView);
        display.update();
    }

    public void periodic() {
        display.update();
    }
}
