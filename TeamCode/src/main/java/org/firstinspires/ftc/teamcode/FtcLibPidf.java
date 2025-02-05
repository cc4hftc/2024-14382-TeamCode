package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class FtcLibPidf extends LinearOpMode {
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    private final double ticks_in_degree = 288 / 180.0;

    private DcMotorEx wrist;

    @Override
    public void runOpMode() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        wrist = hardwareMap.get(DcMotorEx.class, "WristMotor");

        waitForStart();

        while (opModeIsActive()) {
            controller.setPID(p, i, d);
            int wristPos = wrist.getCurrentPosition();
            double pid = controller.calculate(wristPos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

            double power = pid + ff;

            wrist.setPower(power);

            telemetry.addData("Pos ", wristPos);
            telemetry.addData("Target ", target);
            telemetry.update();
        }
    }
}
