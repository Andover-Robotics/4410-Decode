package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Test 1, 2, or 3 servos simultaneously
 *
 * Controls:
 * Left Bumper/Right Bumper - Cycle active servo count (1, 2, or 3)
 * A - Set position 0.5 (middle)
 * B - Set position 0.0
 * Y - Set position 1.0
 * DPad Up/Down   - +/- 0.01
 * DPad Right/Left - +/- 0.001
 */
@TeleOp(name="Multi Servo Test", group="Testing")
public class MultiServoTestV2 extends OpMode {

    private Servo servo1;
    private Servo servo2;
    private Servo servo3;

    private double position = 0.27;
    private int activeCount = 1;

    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadLeft = false;

    @Override
    public void init() {
        servo1 = hardwareMap.get(Servo.class, "leftKicker");
        servo2 = hardwareMap.get(Servo.class, "backKicker");
        servo3 = hardwareMap.get(Servo.class, "rightKicker");

        servo1.setPosition(position);
        servo2.setPosition(position);
        servo3.setPosition(position);

        telemetry.addData("Status", "Initialized - 3 servos at 0.500");
        telemetry.addData("", "Use LB/RB to select how many servos to drive");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Cycle active servo count
        boolean lb = gamepad1.left_bumper;
        boolean rb = gamepad1.right_bumper;

        if (rb && !lastRightBumper) {
            activeCount++;
            if (activeCount > 3) activeCount = 1;
        }
        if (lb && !lastLeftBumper) {
            activeCount--;
            if (activeCount < 1) activeCount = 3;
        }

        lastLeftBumper = lb;
        lastRightBumper = rb;

        // Preset positions
        if (gamepad1.a) {
            position = 0.27;
        } else if (gamepad1.b) {
            position = 0.23;
        } else if (gamepad1.y) {
            position = 0.4;
        }

        // Fine adjustment
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        boolean dpadRight = gamepad1.dpad_right;
        boolean dpadLeft = gamepad1.dpad_left;

        if (dpadUp && !lastDpadUp) position += 0.01;
        if (dpadDown && !lastDpadDown) position -= 0.01;
        if (dpadRight && !lastDpadRight) position += 0.001;
        if (dpadLeft && !lastDpadLeft) position -= 0.001;

        lastDpadUp = dpadUp;
        lastDpadDown = dpadDown;
        lastDpadRight = dpadRight;
        lastDpadLeft = dpadLeft;

        position = Math.max(0.0, Math.min(1.0, position));

        // Apply to active servos
        servo1.setPosition(position);
        if (activeCount >= 2) servo2.setPosition(position);
        if (activeCount >= 3) servo3.setPosition(position);

        // Telemetry
        telemetry.addData("=== MULTI SERVO TEST ===", "");
        telemetry.addData("Active Servos", "%d of 3", activeCount);
        telemetry.addData("Position", "%.3f", position);
        telemetry.addData("", "");
        telemetry.addData("servo1", activeCount >= 1 ? String.format("%.3f", position) : "OFF");
        telemetry.addData("servo2", activeCount >= 2 ? String.format("%.3f", position) : "OFF");
        telemetry.addData("servo3", activeCount >= 3 ? String.format("%.3f", position) : "OFF");
        telemetry.addData("", "");
        telemetry.addData("LB / RB", "Change active servo count");
        telemetry.addData("A", "Middle (0.5)");
        telemetry.addData("B", "Zero (0.0)");
        telemetry.addData("Y", "Full (1.0)");
        telemetry.addData("DPad Up/Down", "+/- 0.01");
        telemetry.addData("DPad Right/Left", "+/- 0.001");
        telemetry.update();
    }
}