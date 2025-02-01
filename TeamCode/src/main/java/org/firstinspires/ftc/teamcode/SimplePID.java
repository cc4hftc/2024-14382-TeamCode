package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp

public class SimplePID extends LinearOpMode {
    private DcMotor wrist = null;

    private double targetPosition = 0;

    private double currentPosition = 0;

    private double motorPower = 0;

    private double Speed = 5;

    @Override
    public void runOpMode() {
        wrist = hardwareMap.get(DcMotor.class, "WristMotor");
        wrist.setDirection(DcMotor.Direction.FORWARD);
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Current", currentPosition);
            telemetry.addData("Target", targetPosition);
            telemetry.addData("Stick", gamepad2.right_stick_y);
            telemetry.update();

            currentPosition = wrist.getCurrentPosition();

            if (gamepad2.right_stick_y < -0.1) {
                targetPosition += 1;
                sleep(10);
            } else if (gamepad2.right_stick_y > 0.1) {
                targetPosition -= 1;
                sleep(10);
            }

            PID();
        }
    }

    private void PID() {
        motorPower = ((targetPosition - currentPosition)/Speed);

        wrist.setPower(motorPower);
    }
}
