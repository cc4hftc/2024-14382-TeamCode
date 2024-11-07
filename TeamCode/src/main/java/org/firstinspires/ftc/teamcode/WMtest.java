package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class WMtest extends LinearOpMode {
    // Drive motors
    DcMotor leftFrontDrive = null;
    DcMotor leftBackDrive = null;
    DcMotor rightFrontDrive = null;
    DcMotor rightBackDrive = null;

    // Arm motors
    DcMotor armMotor = null;
    DcMotor otherArmMotor = null;
    Servo claw = null; // Claw servo reference
    Servo wrist = null;

    // Drive control constants
    private static final double TURN_SPEED_FACTOR = 0.45;
    private static final double ACCELERATION_RATE = 0.08; // Change in power per update
    private static final double strafe_speed = 0.8;
    private static final double SPEED_MULTIPLIER = 1; // Adjust this value to change speed

    // Arm control constants
    private static final int ARM_MIN_POSITION = 2000;  // Adjust based on actual arm configuration
    private static final int ARM_MAX_POSITION = 2000;// Adjust based on actual arm configuration

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

        // Initialize the claw servo (make sure the name matches the configuration)
        claw = hardwareMap.get(Servo.class, "clawServo"); // Make sure the name here matches the one in configuration
        wrist = hardwareMap.get(Servo.class, "wristServo");

        // Set drive motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set arm motor directions
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        otherArmMotor.setDirection(DcMotor.Direction.REVERSE);
        claw.setDirection(Servo.Direction.FORWARD);

        // Set motor modes
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        otherArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            telemetry.clearAll();
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


            // Get the current position of the motor
            int ArmCurrentPosition = armMotor.getCurrentPosition();
            int ArmTargetPos = 0;
            int ArmError = ArmTargetPos - ArmCurrentPosition;

            double kP = 0.1;
            double ArmResistancePower = ArmError * kP;

            ArmResistancePower = Math.max(-1, Math.min(1, ArmResistancePower));
            armMotor.setPower(ArmResistancePower);

            ArmTargetPos = (int) (ArmTargetPos+gamepad2.left_stick_y);


            double newWrist;
            newWrist = (int) (ArmTargetPos+gamepad2.left_stick_y);
            wrist.setPosition(newWrist);

            int clawON = -1;
            double newClaw = 0;
            claw.setPosition(newClaw);
            if (gamepad2.left_bumper || gamepad2.right_bumper) {
                clawON = (clawON *-1);
            }

            if (clawON == -1) {
                newClaw = 0;
            } else {
                newClaw = 150;
            }

            //Debug stuffs
            telemetry.addData("Arm Data:",3);
            telemetry.addData("Current Arm Pos", ArmCurrentPosition);
            telemetry.addData("Target Arm Pos", ArmTargetPos);
            telemetry.addData("ArmResistancePower", ArmResistancePower);
            telemetry.addData("Grabby Data:",2);
            telemetry.addData("Claw Pos", newClaw);
            telemetry.addData("Wrist Pos", newWrist);
            telemetry.addData("Chassis",4);
            telemetry.addData("Front Power", leftFrontPower);
            telemetry.addData("Back Power", rightFrontPower);
            telemetry.addData("Left Power", leftBackPower);
            telemetry.addData("Right Power", rightBackPower);
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
