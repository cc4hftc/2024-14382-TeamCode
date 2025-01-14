package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class TeleOP_MAIN extends LinearOpMode {

    // Drive motors
    DcMotor leftFrontDrive = null;
    DcMotor leftBackDrive = null;
    DcMotor rightFrontDrive = null;
    DcMotor rightBackDrive = null;

    // Arm motors
    DcMotorEx armMotor = null;
    DcMotorEx otherArmMotor = null;
    Servo claw = null; // Claw servo reference
    Servo other_claw = null; // Claw servo reference
    DcMotor winch = null;

    // PID control constants for the arm
    private double otherTargetPosition;
    private double targetPosition;
    private double kP = 0.0001; //0.001
    private double kI = 0.00;
    private double kD = 0.000;
    private double kF = 0.0025;
    private double integral = 0;
    private double lastError = 0;
    private double otherlastError = 0;
    private ElapsedTime timer = new ElapsedTime();
    private boolean rightTriggerEnabled = true;
    private boolean leftTriggerEnabled = true;

    // Drive control constants
    private static final double TURN_SPEED_FACTOR = 0.35;
    private static final double ACCELERATION_RATE = 0.08; // Change in power per update
    private static final double strafe_speed = 0.8;
    private static final double SPEED_MULTIPLIER = 1; // Adjust this value to change speed

    // Power variables for the drive system
    private double leftFrontPower = 0;
    private double leftBackPower = 0;
    private double rightFrontPower = 0;
    private double rightBackPower = 0;

    // Claw control
    private double newWrist = 0;
    Servo wrist = null;
    private double newClaw = 0.1655;

    @Override
    public void runOpMode() {
        // Initialize drive motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");    //Port 0
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");      // Port 1
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");  // Port 2
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");    // Port 3

        // Initialize arm motors
        armMotor = hardwareMap.get(DcMotorEx.class, "arm_motor");               // Port 2
        otherArmMotor = hardwareMap.get(DcMotorEx.class, "arm_motor2");         // Port 1

        // Initialize the claw servo (make sure the name matches the configuration)
        claw = hardwareMap.get(Servo.class, "clawServo");                       // Port 1
        other_claw = hardwareMap.get(Servo.class, "other_clawServo");           // Port 2
        wrist = hardwareMap.get(Servo.class, "wristServo");                     // Port 0
        winch = hardwareMap.get(DcMotor.class, "winch");                        // Port 0

        // Set drive motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set arm motor directions
        armMotor.setDirection(DcMotorEx.Direction.FORWARD);
        otherArmMotor.setDirection(DcMotorEx.Direction.REVERSE);
        claw.setDirection(Servo.Direction.FORWARD);
        other_claw.setDirection(Servo.Direction.REVERSE);
        winch.setDirection(DcMotor.Direction.FORWARD);


        // Set motor modes
        otherArmMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        otherArmMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        otherTargetPosition = 0;
        targetPosition = 0;
        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            //int otherArmpos = otherArmMotor.getCurrentPosition();
            //int Armpos = armMotor.getCurrentPosition();
            armMotor.setTargetPosition(otherArmMotor.getTargetPosition());
            telemetry.clearAll();
            // Omni-wheel drive control (gamepad1)
            double drive = -gamepad1.left_stick_y * SPEED_MULTIPLIER;  // Forward/backward
            double strafe = gamepad1.right_stick_x * strafe_speed * SPEED_MULTIPLIER;  // Left/right
            double rotate = (gamepad1.right_trigger-gamepad1.left_trigger) * TURN_SPEED_FACTOR * SPEED_MULTIPLIER;

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
            double armPower = gamepad2.left_trigger - gamepad2.right_trigger;


            //ARM CODE//ARM CODE//ARM CODE//ARM CODE//ARM CODE//ARM CODE//ARM CODE//ARM CODE//

            if (rightTriggerEnabled && armPower > 0.1) {
                // Right trigger is pressed (move arm upwards)
                armMotor.setPower(armPower / 12);
                otherArmMotor.setPower(armPower / 12);
                otherTargetPosition = otherArmMotor.getCurrentPosition();
                targetPosition = armMotor.getCurrentPosition();
                resetPID();
            } else if (leftTriggerEnabled && armPower < -0.1) {
                // Left trigger is pressed (move arm downwards)
                armMotor.setPower(armPower / 12);
                otherArmMotor.setPower(armPower / 12);
                otherTargetPosition = otherArmMotor.getCurrentPosition();
                targetPosition = armMotor.getCurrentPosition();
                resetPID();
            } else {
                // No trigger pressed: PID control
                pidControl();
            }


            double tick = otherArmMotor.getCurrentPosition();

            /*if (gamepad1.a) {
                winch.setPower(0.5);
            } else if (gamepad1.b) {
                winch.setPower(-0.5);
            } else {
                winch.setPower(0);
            }*/

            if (gamepad1.y) {
                armMotor.setPower(0);
                otherArmMotor.setPower(0);
                resetPID();
            }

            // Stop the right trigger if tick exceeds 410
            if (tick >= 375) {
                rightTriggerEnabled = false;
            } else if (tick >= 1 && tick <= 374) {
                rightTriggerEnabled = true;
                leftTriggerEnabled = true;
            } else if (tick <= 0) {
                leftTriggerEnabled = false;
                rightTriggerEnabled = true;
            }

            //Set the wrist target position
            newWrist += (gamepad2.right_stick_y/150);

            //Set the min and max wrist positions
            newWrist = Math.max(0, Math.min(0.72, newWrist)); // 0.72

            //Set the wrist servo position
            wrist.setPosition(newWrist);


            //CLAW CODE//CLAW CODE//CLAW CODE//CLAW CODE//CLAW CODE//CLAW CODE//CLAW CODE//

            //Detect a bumper press
            if (gamepad2.right_bumper) {  //Close
                newClaw = 0.1775;
            } else if (gamepad2.left_bumper) {  //Open
                newClaw = 0.08;
            }

            //Set the claw servo position
            claw.setPosition(newClaw);
            other_claw.setPosition(newClaw);



            //DEBUG CODE//DEBUG CODE//DEBUG CODE//DEBUG CODE//DEBUG CODE//DEBUG CODE//DEBUG CODE//

            //Arm debug
            telemetry.addData("--Arm Data:::",4);
            telemetry.addData("  - Current otherArm Pos", otherArmMotor.getCurrentPosition());
            telemetry.addData("  - Current Arm pos", armMotor.getCurrentPosition());
            telemetry.addData("  - Target Arm Pos", targetPosition);
            telemetry.addData("  - Other Target Arm Pos", otherTargetPosition);
            telemetry.addData(" ", null);

            //PIDF debug
            telemetry.addData("--PIDF Data:::",4);
            telemetry.addData("  - kP", kP);
            telemetry.addData("  - kI", kI);
            telemetry.addData("  - kD", kD);
            telemetry.addData("  - kF", kF);
            telemetry.addData(" ", null);

            //Grabby debug
            telemetry.addData("--Grabby Data:::",2);
            telemetry.addData("  - Claw Pos", newClaw);
            telemetry.addData("  - Wrist Pos", newWrist);
            telemetry.addData(" ", null);

            //Chassis debug
            telemetry.addData("--Chassis:::",4);
            telemetry.addData("  - Front Power", leftFrontPower);
            telemetry.addData("  - Back Power", rightFrontPower);
            telemetry.addData("  - Left Power", leftBackPower);
            telemetry.addData("  - Right Power", rightBackPower);
            telemetry.addData("  - Front Position", rightFrontDrive.getCurrentPosition());
            telemetry.addData("  - Back Position", leftFrontDrive.getCurrentPosition());
            telemetry.addData("  - Left Position", leftBackDrive.getCurrentPosition());
            telemetry.addData("  - Right Position", rightBackDrive.getCurrentPosition());
            telemetry.addData(" ", null);
            telemetry.update();



        }
    }

    private void pidControl() {
        double otherCurrentPosition = otherArmMotor.getCurrentPosition();
        double currentPosition = armMotor.getCurrentPosition();
        double otherError = otherTargetPosition - otherCurrentPosition;
        double error = targetPosition - currentPosition;
        double deltaTime = timer.seconds();
        integral += otherError * deltaTime;
        double derivative = (otherError - lastError) / deltaTime;
        double power = kP * error + kI * integral + kD * derivative + kF;
        double otherPower = kP * otherError + kI * integral + kD * derivative + kF;

        // Apply mirrored power to both motors
        armMotor.setPower(power);
        otherArmMotor.setPower(otherPower);

        lastError = error;
        otherlastError = otherError;
        timer.reset();
    }


    private void resetPID() {
        integral = 0;
        lastError = 0;
        otherlastError = 0;
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
