package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class OmniWheelTeleOp extends LinearOpMode {
    DcMotor leftFrontDrive = null;
    DcMotor leftBackDrive = null;
    DcMotor rightFrontDrive = null;
    DcMotor rightBackDrive = null;

    // Turn speed factor (0.5 means half speed for turning)
    private static final double TURN_SPEED_FACTOR = 0.5;

    @Override
    public void runOpMode() {
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Optionally set motor directions if needed
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            // Get joystick input for driving
            double drive = -gamepad1.left_stick_y;  // Forward/backward
            double strafe = gamepad1.right_stick_x;  // Left/right

            // Use triggers for rotation
            double rotate = (gamepad1.right_trigger - gamepad1.left_trigger) * TURN_SPEED_FACTOR;

            // Calculate power for each motor
            double leftFrontPower = -strafe + rotate;
            double leftBackPower = drive + rotate;
            double rightFrontPower = -strafe - rotate;
            double rightBackPower = drive - rotate;

            // Normalize the motor power values to ensure they stay within [-1, 1]
            double maxPower = Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(leftBackPower),
                    Math.max(Math.abs(rightFrontPower), Math.abs(rightBackPower))));
            if (maxPower > 1) {
                leftFrontPower /= maxPower;
                leftBackPower /= maxPower;
                rightFrontPower /= maxPower;
                rightBackPower /= maxPower;
            }

            // Set power to the motors
            leftFrontDrive.setPower(leftFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightFrontDrive.setPower(rightFrontPower);
            rightBackDrive.setPower(rightBackPower);
        }
    }
}
