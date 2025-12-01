package org.firstinspires.ftc.teamcode.led;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "LED Matrix Demo", group = "Examples")
public class LedMatrixDemoOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        LedMatrixController matrix = new LedMatrixController(hardwareMap, "i2cMatrix");
        LedMatrixBuffer buffer = matrix.getBuffer();

        // Prepare a startup pattern
        buffer.clear();
        buffer.drawRectangle(0, 0, LedMatrixBuffer.ROWS, LedMatrixBuffer.COLS, 16, false);
        buffer.drawCircle(3, 5, 3, 32, true);
        buffer.drawDigit(1, 10, 2, 48);
        buffer.drawDigit(1, 14, 4, 48);
        buffer.drawDigit(1, 18, 1, 48);
        matrix.update();

        waitForStart();

        while (opModeIsActive()) {
            // Pulse the border brightness based on runtime
            int brightness = (int) ((Math.sin(getRuntime()) * 0.5 + 0.5) * 255);
            buffer.clear();
            buffer.drawRectangle(0, 0, LedMatrixBuffer.ROWS, LedMatrixBuffer.COLS, brightness, false);
            buffer.drawCircle(3, 5, 3, brightness / 2, true);
            buffer.drawDigit(1, 10, (int) (getRuntime()) % 10, 255);
            matrix.update();

            telemetry.addData("LED border", brightness);
            telemetry.update();
            sleep(50);
        }
    }
}
