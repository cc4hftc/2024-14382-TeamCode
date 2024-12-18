package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class TestBed extends LinearOpMode {

    // Arm motor initialization
    DcMotor armMotor = null;
    DcMotor otherArmMotor = null;

    // PIDF control constants for the arm
    private double targetPosition;
    private double kP = 0.0001; //0.001
    private double kI = 0.00;
    private double kD = 0.000;
    private double kF = 0.0025;
    private double integral = 0;
    private double lastError = 0;
    private ElapsedTime timer = new ElapsedTime();
    private boolean rightTriggerEnabled = true;
    private boolean leftTriggerEnabled = true;


    // Drive initialization
    DcMotor leftFrontDrive = null;
    DcMotor leftBackDrive = null;
    DcMotor rightFrontDrive = null;
    DcMotor rightBackDrive = null;

    // Drive variables
    private double leftFrontPower = 0;
    private double leftBackPower = 0;
    private double rightFrontPower = 0;
    private double rightBackPower = 0;

    // Drive control constants
    private static final double TURN_SPEED_FACTOR = 0.45;
    private static final double ACCELERATION_RATE = 0.08; // Change in power per update
    private static final double STRAFE_SPEED = 0.8;
    private static final double DRIVE_SPEED = 1; // Adjust this value to change speed


    // Claw variables/initialization
    Servo claw;
    private double newClaw = 0.1655;


    // Wrist variables/initialization
    Servo wrist = null;
    private double newWrist = 0;


    @Override
    public void runOpMode() {

        // Initialize drive motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Set drive motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);


        // Initialize arm motors
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        otherArmMotor = hardwareMap.get(DcMotor.class, "arm_motor2");

        // Set arm motor directions
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        otherArmMotor.setDirection(DcMotor.Direction.REVERSE);
        claw.setDirection(Servo.Direction.FORWARD);

        // Initialize the claw servo
        claw = hardwareMap.get(Servo.class, "clawServo");
        wrist = hardwareMap.get(Servo.class, "wristServo");

        // Set motor modes
        otherArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        otherArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        targetPosition = 0;
        waitForStart();
        timer.reset();

        while (opModeIsActive()) {

            telemetry.clearAll();
            // Omni-wheel drive control (gamepad1)
            double drive = -gamepad1.left_stick_y * DRIVE_SPEED;  // Forward/backward
            double strafe = gamepad1.right_stick_x * STRAFE_SPEED * DRIVE_SPEED;  // Left/right
            double rotate = (gamepad1.right_trigger - gamepad1.left_trigger) * TURN_SPEED_FACTOR * DRIVE_SPEED;

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


            //ARM CODE//ARM CODE//ARM CODE//ARM CODE//ARM CODE//ARM CODE//ARM CODE//ARM CODE//

            if (rightTriggerEnabled && armPower > 0.1) {
                // Right trigger is pressed (move arm upwards)
                armMotor.setPower(armPower / 15);
                otherArmMotor.setPower(armPower / 15);
                targetPosition = otherArmMotor.getCurrentPosition();
                resetPID();
            } else if (leftTriggerEnabled && armPower < -0.1) {
                // Left trigger is pressed (move arm downwards)
                armMotor.setPower(armPower / 15);
                otherArmMotor.setPower(armPower / 15);
                targetPosition = otherArmMotor.getCurrentPosition();
                resetPID();
            } else {
                // No trigger pressed: PID control
                pidControl();
            }

            double tick = otherArmMotor.getCurrentPosition();

            // Stop the right trigger if tick exceeds 1000
            if (tick >= 500) {
                rightTriggerEnabled = false;
            } else if (tick >= 1 && tick <= 499) {
                rightTriggerEnabled = true;
                leftTriggerEnabled = true;
            } else if (tick <= 0) {
                leftTriggerEnabled = false;
                rightTriggerEnabled = true;
            }


            //WRIST CODE//WRIST CODE//WRIST CODE//WRIST CODE//WRIST CODE//WRIST CODE//WRIST CODE//

            //Set the wrist target position
            newWrist += (gamepad2.right_stick_y/300);

            //Set the min and max wrist positions
            newWrist = Math.max(0, Math.min(0.75, newWrist));

            //Set the wrist servo position
            wrist.setPosition(newWrist);


            //CLAW CODE//CLAW CODE//CLAW CODE//CLAW CODE//CLAW CODE//CLAW CODE//CLAW CODE//

            //Detect a bumper press
            if (gamepad2.left_bumper) {  //Close
                newClaw = 0.1655;
            } else if (gamepad2.right_bumper) {  //Open
                newClaw = 0;
            }

            //Set the claw servo position
            claw.setPosition(newClaw);


            //DEBUG CODE//DEBUG CODE//DEBUG CODE//DEBUG CODE//DEBUG CODE//DEBUG CODE//DEBUG CODE//

            //Arm debug
            telemetry.addData("--Arm Data:::","WIP");
            telemetry.addData("  - Current Arm Pos", otherArmMotor.getCurrentPosition());
            telemetry.addData("  - Target Arm Pos", targetPosition);

            //PIDF debug
            telemetry.addData("--PIDF Data:::","WIP'");
            telemetry.addData("  - kP", kP);
            telemetry.addData("  - kI", kI);
            telemetry.addData("  - kD", kD);
            telemetry.addData("  - kF", kF);

            //Grabby debug
            telemetry.addData("--Grabby Data:::",2);
            telemetry.addData("  - Claw Pos", newClaw);
            telemetry.addData("  - Wrist Pos", newWrist);

            //Chassis debug
            telemetry.addData("--Chassis:::",4);
            telemetry.addData("  - Front Power", leftFrontPower);
            telemetry.addData("  - Back Power", rightFrontPower);
            telemetry.addData("  - Left Power", leftBackPower);
            telemetry.addData("  - Right Power", rightBackPower);
            telemetry.update();



        }
    }

    private void pidControl() {
        double currentPosition = otherArmMotor.getCurrentPosition();
        double error = targetPosition - currentPosition;
        double deltaTime = timer.seconds();
        integral += error * deltaTime;
        double derivative = (error - lastError) / deltaTime;
        double power = kP * error + kI * integral + kD * derivative + kF;

        armMotor.setPower(power);
        otherArmMotor.setPower(power);

        lastError = error;
        timer.reset();
    }

    private void resetPID() {
        integral = 0;
        lastError = 0;
        timer.reset();
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
