package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class OUTREACH_TeleOP_KID extends LinearOpMode {

    // Drive motors
    DcMotor leftFrontDrive = null;
    DcMotor leftBackDrive = null;
    DcMotor rightFrontDrive = null;
    DcMotor rightBackDrive = null;

    // Drive control constants
    private static final double TURN_SPEED_FACTOR = 0.45;
    private static final double ACCELERATION_RATE = 0.08; // Change in power per update
    private static final double strafe_speed = 0.8;
    private static final double SPEED_MULTIPLIER = 1; // Adjust this value to change speed

    // Power variables for the drive system
    private double leftFrontPower = 0;
    private double leftBackPower = 0;
    private double rightFrontPower = 0;
    private double rightBackPower = 0;

    int OVERRIDE = 0;

    double drive = 0;
    double strafe = 0;
    double rotate = 0;

    @Override
    public void runOpMode() {
        // Initialize drive motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");    //Port 0
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");      // Port 1
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");  // Port 2
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");    // Port 3

        // Set drive motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the start signal
        waitForStart();  // Fix: Wait for start signal from the driver station

        while (opModeIsActive()) {

            telemetry.clearAll();

            if(OVERRIDE == 0) {
                // Omni-wheel drive control (gamepad1)
                drive = -gamepad1.left_stick_y * SPEED_MULTIPLIER;  // Forward/backward
                strafe = gamepad1.right_stick_x * strafe_speed * SPEED_MULTIPLIER;  // Left/right
                rotate = (gamepad1.right_trigger - gamepad1.left_trigger) * TURN_SPEED_FACTOR * SPEED_MULTIPLIER;
            } else if(OVERRIDE == 1) {
                // Omni-wheel drive control (gamepad1)
                drive = -gamepad2.left_stick_y * SPEED_MULTIPLIER;  // Forward/backward
                strafe = gamepad2.right_stick_x * strafe_speed * SPEED_MULTIPLIER;  // Left/right
                rotate = (gamepad2.right_trigger - gamepad2.left_trigger) * TURN_SPEED_FACTOR * SPEED_MULTIPLIER;
            }

            if(gamepad2.x) {
                OVERRIDE = 1;
            } else {
                OVERRIDE = 0;
            }

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

            //DEBUG CODE//DEBUG CODE//DEBUG CODE//DEBUG CODE//DEBUG CODE//DEBUG CODE//DEBUG CODE//

            //KID debug
            telemetry.addData("THIS IS A KID DRIVER CODE", null);
            telemetry.addData(" ", null);

            //Chassis debug
            telemetry.addData("--Chassis:::", 4);
            telemetry.addData("  - Front Power", leftFrontPower);
            telemetry.addData("  - Back Power", rightFrontPower);
            telemetry.addData("  - Left Power", leftBackPower);
            telemetry.addData("  - Right Power", rightBackPower);
            telemetry.addData(" ", null);
            telemetry.addData(" ", null);

            if(OVERRIDE == 0) {
                telemetry.addData("--KID  CONTROL  ACTIVE--", null);
            } else if(OVERRIDE == 1) {
                telemetry.addData("--OVERRIDE  ACTIVE--", null);
            }

            telemetry.update();
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