package org.firstinspires.ftc.teamcode.teleop.subsystems.screen;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.subsystems.screen.TestView;

import team.techtigers.core.display.AdafruitNeoPixel;
import team.techtigers.core.display.VisualDisplay;

@TeleOp(name = "Visual Display Test Op Mode", group="Test")
public class VisualDisplayTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AdafruitNeoPixel driver = hardwareMap.get(AdafruitNeoPixel.class, "screen");
        driver.initialize(192,3);
        TestView view = new TestView();
        VisualDisplay visualDisplay = new VisualDisplay(driver, view);

        waitForStart();

        while(opModeIsActive()){
            visualDisplay.update();
        }
    }
}
