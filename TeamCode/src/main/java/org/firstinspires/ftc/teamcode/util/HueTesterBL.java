package org.firstinspires.ftc.teamcode.util;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * Analog HSV Tester
 *
 * Reads voltage from analog port "pin0" and converts to hue:
 * hue = (voltage / 3.3) * 360
 */
@TeleOp(name = "Analog Hue Test", group = "Sensor Test")
public class HueTesterBL extends LinearOpMode {

    @Override
    public void runOpMode() {

        AnalogInput pin0 = hardwareMap.get(AnalogInput.class, "pin0");

        telemetry.addLine("Analog Hue Test Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double voltage = pin0.getVoltage();
            double hue = voltage / 3.3 * 360;

            telemetry.addData("Voltage", "%.3f V", voltage);
            telemetry.addData("Hue", "%.2f deg", hue);

            telemetry.update();
        }
    }
}