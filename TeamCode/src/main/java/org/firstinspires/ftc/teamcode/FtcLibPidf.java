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

    public int StickPosition = 0;

    private final double ticks_in_degree = 288 / 180.0;

    private DcMotorEx wrist;

    @Override
    public void runOpMode() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        wrist = hardwareMap.get(DcMotorEx.class, "WristMotor");


        waitForStart();

        while (opModeIsActive()) {
            target = StickPosition;
            controller.setPID(p, i, d);
            int wristPos = wrist.getCurrentPosition();
            double pid = controller.calculate(wristPos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

            double power = pid + ff;

            StickToInt();

            //WristLimit(150);

            wrist.setPower(power);

            telemetry.addData("Pos ", wristPos);
            telemetry.addData("Target ", target);
            telemetry.update();
        }
    }

    private void StickToInt() {
        double stick = gamepad2.right_stick_y;

        if (stick < -0.1) {
            StickPosition += 1;
            sleep(20);
        } else if (stick > 0.1) {
            StickPosition -= 1;
            sleep(20);
        } else if (stick == 0) {
            // Do nothing
        }
    }

    private void WristLimit(int limit) {
        if (wrist.getCurrentPosition() >= limit) {
            wrist.setTargetPosition(limit-1);
        } /*else if (wrist.getCurrentPosition() <= 0) {
            wrist.setTargetPosition(1);
        }*/
    }
}
