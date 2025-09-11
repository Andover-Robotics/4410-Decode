package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensorMultiplexer;

@Config
@TeleOp
public class PIDF extends OpMode {
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    private final double ticks_in_degree = 360 / 145.1; // 1150 RPM Motor

    private DcMotorEx arm_motor;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turret_motor = hardwareMap.get(DcMotorEx.class, "turret_motor0");

    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int turretPos = turret_motor.getCurrentPosition();
        double pid = controller.calculate(turretPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        turret_motor.setPower(power);

        telemetry.addData("pos ", turretPos);
        telemetry.addData("target ", target);
        telemetry.update();
    }
}
