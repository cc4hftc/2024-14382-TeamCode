package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class OmniWheelTeleOp extends LinearOpMode {
    // Drive motors
    DcMotor leftFrontDrive = null;
    DcMotor leftBackDrive = null;
    DcMotor rightFrontDrive = null;
    DcMotor rightBackDrive = null;

    // Arm motors
    DcMotor armMotor = null;
    DcMotor otherArmMotor = null;

    // Drive control constants
    private static final double TURN_SPEED_FACTOR = 0.7;
    private static final double ACCELERATION_RATE = 0.08; // Change in power per update
    private static final double strafe_speed = 0.8;
    private static final double SPEED_MULTIPLIER = 1; // Adjust this value to change speed

    // Arm control constants
    private static final int ARM_MIN_POSITION = 0;  // Adjust based on actual arm configuration
    private static final int ARM_MAX_POSITION = 180;  // Adjust based on actual arm configuration

    // Power variables for the drive system
    private double leftFrontPower = 0;
    private double leftBackPower = 0;
    private double rightFrontPower = 0;
    private double rightBackPower = 0;

    @Override
    public void runOpMode() {
        // Initialize drive motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Initialize arm motors
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        otherArmMotor = hardwareMap.get(DcMotor.class, "arm_motor2");

        // Set drive motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set arm motor directions
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        otherArmMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set motor modes
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        otherArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            // Omni-wheel drive control (gamepad1)
            double drive = -gamepad1.left_stick_y * SPEED_MULTIPLIER;  // Forward/backward
            double strafe = gamepad1.right_stick_x * strafe_speed * SPEED_MULTIPLIER;  // Left/right
            double rotate = (gamepad1.right_trigger - gamepad1.left_trigger) * TURN_SPEED_FACTOR * SPEED_MULTIPLIER;

            // Calculate target power for each drive motor
            double targetLeftFrontPower = -strafe + rotate;
            double targetLeftBackPower = drive + rotate;
            double targetRightFrontPower = -strafe - rotate;
            double targetRightBackPower = drive - rotate;

            // Ramp up the power for smoother acceleration
            leftFrontPower = rampPower(leftFrontPower, targetLeftFrontPower);
            leftBackPower = rampPower(leftBackPower, targetLeftBackPower);
            rightFrontPower = rampPower(rightFrontPower, targetRightFrontPower);
            rightBackPower = rampPower(rightBackPower, targetRightBackPower);

            // Set power to the drive motors
            leftFrontDrive.setPower(leftFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightFrontDrive.setPower(rightFrontPower);
            rightBackDrive.setPower(rightBackPower);

            // Arm control (gamepad2)
            double armPower = gamepad2.right_trigger - gamepad2.left_trigger;
            int currentPosition = armMotor.getCurrentPosition();

            // Move the arm based on trigger input with position limits
            if (armPower > 0 && currentPosition < ARM_MAX_POSITION) {
                armMotor.setPower(armPower);
                otherArmMotor.setPower(armPower);
            } else if (armPower < 0 && currentPosition > ARM_MIN_POSITION) {
                armMotor.setPower(armPower);
                otherArmMotor.setPower(armPower);
            } else {
                armMotor.setPower(0);
                otherArmMotor.setPower(0);
            }

            // Button-based arm control (gamepad2)
            if (gamepad2.a) {
                // Move arm up if not at max position
                if (currentPosition < ARM_MAX_POSITION) {
                    armMotor.setPower(1);
                    otherArmMotor.setPower(1);
                }
            } else if (gamepad2.b) {
                // Move arm down if not at min position
                if (currentPosition > ARM_MIN_POSITION) {
                    armMotor.setPower(-1);
                    otherArmMotor.setPower(-1);
                }
            } else {
                armMotor.setPower(0); // Stop the arm
                otherArmMotor.setPower(0); // Stop the other arm
            }

            // Optional small delay to prevent rapid updates (gamepad2)
            sleep(50);
        }
    }

    // Helper function to ramp motor power smoothly
    private double rampPower(double currentPower, double targetPower) {
        if (currentPower < targetPower) {
            currentPower += ACCELERATION_RATE;
            if (currentPower > targetPower) {
                currentPower = targetPower;
            }
        } else if (currentPower > targetPower) {
            currentPower -= ACCELERATION_RATE;
            if (currentPower < targetPower) {
                currentPower = targetPower;
            }
        }
        return currentPower;
    }
}
