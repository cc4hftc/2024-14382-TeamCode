package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class PID extends LinearOpMode {
    DcMotor wrist = null;

    private ElapsedTime timer = new ElapsedTime();

    private double kP = 0.005;
    private double kI = 0.0;
    private double kD = 0.0;
    private double kF = 0.005;
    private double TargetPosition;
    private double integral = 0;
    private double lastError = 0;
    private int StickPosition = 0;

    @Override
    public void runOpMode() {
        wrist = hardwareMap.get(DcMotor.class, "WristMotor");

        wrist.setDirection(DcMotor.Direction.FORWARD);

        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {

            telemetry.addData("  - Wrist Pos", wrist.getCurrentPosition());
            telemetry.addData("  - Wrist Target", TargetPosition);
            telemetry.addData("  - Stick", gamepad2.right_stick_y);
            telemetry.addData("  - Integral", integral);
            telemetry.addData("  - StickPosition", StickPosition);
            telemetry.update();

            if (StickPosition > 0) {
                // Manual wrist movement
                wrist.setTargetPosition(StickPosition);
                TargetPosition = wrist.getTargetPosition();
                resetWristPID();
            }

            WristPID();

            StickToInt();
        }
    }

    private void WristPID() {
        double CurrentPosition = wrist.getCurrentPosition();
        double Error = TargetPosition - CurrentPosition;
        double DeltaTime = timer.seconds();
        integral += Error * DeltaTime;
        double WristDerivative = (Error - lastError) / DeltaTime;
        double WristPower = kP * Error + kI * integral + kD * WristDerivative + kF;

        wrist.setPower(WristPower);

        lastError = Error;
        timer.reset();
    }

    private void resetWristPID() {
        integral = 0;
        lastError = 0;
        timer.reset();
    }

    private void StickToInt() {
        if (gamepad2.right_stick_y < 0) {
            StickPosition += 1;
        }
    }
}
