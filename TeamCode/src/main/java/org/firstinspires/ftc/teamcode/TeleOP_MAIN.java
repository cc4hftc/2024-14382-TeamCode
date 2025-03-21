package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
public class TeleOP_MAIN extends LinearOpMode {
    private PIDController WristController;

    public static double wP = 0.025, wI = 0, wD = 0.01;
    public static double wF = 0.075;

    public static long ST = 2;

    public static int limit = 150;

    public static int target = 0;

    public int StickPosition = 0;

    private final double ticks_in_degree = 288 / 180.0;

    // Drive motors
    DcMotor leftFrontDrive = null;
    DcMotor leftBackDrive = null;
    DcMotor rightFrontDrive = null;
    DcMotor rightBackDrive = null;

    // Arm motors
    DcMotorEx armMotor = null;
    DcMotorEx otherArmMotor = null;
    Servo claw = null;
    Servo other_claw = null;

    // PID control constants for the arm
    private double otherTargetPosition;
    private double targetPosition;
    private double WristTargetPosition;
    private double kP = 0.0001; //0.001
    private double kI = 0.00;
    private double kD = 0.000;
    private double kF = 0.0025;
    /*private double wP = 1;
    private double wI = 0.1;
    private double wD = 10;
    private double wF = 0.5;
    private int SitckPosition = 0;*/
    private double integral = 0;
    //private double WristIntegral = 0;
    private double lastError = 0;
    //private double WristLastError = 0;
    private double otherlastError = 0;
    private ElapsedTime timer = new ElapsedTime();
    private boolean rightTriggerEnabled = true;
    private boolean leftTriggerEnabled = true;
    private boolean WristMoveAble = true;

    // Drive control constants
    private static final double TURN_SPEED_FACTOR = 0.35;
    private static final double ACCELERATION_RATE = 0.08;
    private static final double strafe_speed = 0.8;
    private static final double SPEED_MULTIPLIER = 1;

    // Power variables for the drive system
    private double leftFrontPower = 0;
    private double leftBackPower = 0;
    private double rightFrontPower = 0;
    private double rightBackPower = 0;

    // Claw control
    private double newWrist = 0;
    DcMotor wrist = null;
    private double newClaw = 0.1455;

    @Override
    public void runOpMode() {
        WristController = new PIDController(wP, wI, wD);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize drive motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Initialize arm motors
        armMotor = hardwareMap.get(DcMotorEx.class, "arm_motor");
        otherArmMotor = hardwareMap.get(DcMotorEx.class, "arm_motor2");
        claw = hardwareMap.get(Servo.class, "clawServo");
        other_claw = hardwareMap.get(Servo.class, "other_clawServo");
        wrist = hardwareMap.get(DcMotor.class, "WristMotor");

        // Set motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotorEx.Direction.FORWARD);
        otherArmMotor.setDirection(DcMotorEx.Direction.REVERSE);
        claw.setDirection(Servo.Direction.FORWARD);
        other_claw.setDirection(Servo.Direction.REVERSE);
        wrist.setDirection(DcMotor.Direction.FORWARD);

        // Set motor modes
        otherArmMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        otherArmMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wrist.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        otherTargetPosition = 0;
        targetPosition = 0;
        //WristTargetPosition = 0;
        waitForStart();
        timer.reset();

        boolean wristMoveOut = false;

        boolean wristMoveIn = false;

        while (opModeIsActive()) {
            target = StickPosition;

            WristController.setPID(wP, wI, wD);
            int wristPos = wrist.getCurrentPosition();
            double pid = WristController.calculate(wristPos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * wF;

            double power = pid + ff;

            StickToInt();

            wrist.setPower(power);

            //int otherArmpos = otherArmMotor.getCurrentPosition();
            //int Armpos = armMotor.getCurrentPosition();
            armMotor.setTargetPosition(otherArmMotor.getTargetPosition());
            //telemetry.clearAll();
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

            if (gamepad1.dpad_left) {
                leftFrontDrive.setPower(0.2);
                rightFrontDrive.setPower(0.2);
            } else if (gamepad1.dpad_right) {
                leftFrontDrive.setPower(-0.2);
                rightFrontDrive.setPower(-0.2);
            } else if (gamepad1.dpad_up) {
                leftBackDrive.setPower(0.1);
                rightBackDrive.setPower(0.1);
            } else if (gamepad1.dpad_down) {
                leftBackDrive.setPower(-0.1);
                rightBackDrive.setPower(-0.1);
            }

            // Arm control (gamepad2)
            double armPower = gamepad2.left_trigger - gamepad2.right_trigger;

            double WristTicks = wrist.getCurrentPosition();

            double MoveWrist = -gamepad2.right_stick_y;
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

            if (gamepad2.a) {
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                otherArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setTargetPosition(240);
                otherArmMotor.setTargetPosition(240);
                armMotor.setPower(0.35);
                otherArmMotor.setPower(0.35);
                if (armMotor.getCurrentPosition() >= armMotor.getTargetPosition()) {
                    pidControl();
                } else if (otherArmMotor.getCurrentPosition() >= otherArmMotor.getTargetPosition()) {
                    pidControl();
                } else {
                    pidControl();
                }
            }


            double tick = otherArmMotor.getCurrentPosition();


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

            /*if (WristTicks >= 150) {
                WristMoveAble = false;
            } else if (WristTicks >=1 && WristTicks <= 149) {
                WristMoveAble = true;
            } else if (WristTicks <= 0) {
                WristMoveAble = false;
            }*/

            // Wrist control with updated logic
            //final double JOYSTICK_DEADZONE = 0.1;
            //int joystickInput = -gamepad2.right_stick_y;

            /*if (SitckPosition > 0) {
                // Manual wrist movement
                wrist.setTargetPosition(SitckPosition);
                WristTargetPosition = wrist.getTargetPosition();
                resetWristPID();
            }

            WristPID();

            StickToInt();*/

            //CLAW CODE//CLAW CODE//CLAW CODE//CLAW CODE//CLAW CODE//CLAW CODE//CLAW CODE//

            //Detect a bumper press
            if (gamepad2.right_bumper) {  //Close
                newClaw = 0.1435;
            } else if (gamepad2.left_bumper) {  //Open
                newClaw = 0.07;
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
            //telemetry.addData(" ", null);

            //PIDF debug
            telemetry.addData("--PIDF Data:::",4);
            telemetry.addData("  - kP", kP);
            telemetry.addData("  - kI", kI);
            telemetry.addData("  - kD", kD);
            telemetry.addData("  - kF", kF);
            //telemetry.addData(" ", null);

            //Grabby debug
            telemetry.addData("--Grabby Data:::",2);
            telemetry.addData("  - Claw Pos", newClaw);
            telemetry.addData("  - Wrist Pos", wrist.getCurrentPosition());
            telemetry.addData("  - Wrist Target", WristTargetPosition);
            telemetry.addData("  - Stick", gamepad2.right_stick_y);
            telemetry.addData("  - Pos ", wristPos);
            telemetry.addData("  - Target ", target);
            //telemetry.addData("  - WristIntegral", WristIntegral);
            //telemetry.addData("  - SitckPosition", SitckPosition);
            //telemetry.addData(" ", null);

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
            //telemetry.addData(" ", null);
            telemetry.update();
        }
    }

    /*private void WristPID() {
        double WristCurrentPosition = wrist.getCurrentPosition();
        double WristError = WristTargetPosition - WristCurrentPosition;
        double WristDeltaTime = timer.seconds();
        WristIntegral += WristError * WristDeltaTime;
        double WristDerivative = (WristError - WristLastError) / WristDeltaTime;
        double WristPower = wP * WristError + wI * WristIntegral + wD * WristDerivative + wF;

        wrist.setPower(WristPower);

        WristLastError = WristError;
        timer.reset();
    }

    private void resetWristPID() {
        WristIntegral = 0;
        WristLastError = 0;
        timer.reset();
    }*/

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

    private void StickToInt() {
        double stick = gamepad2.right_stick_y;
        if (StickPosition >= limit) {
            StickPosition-=1;
        }

        if (stick > 0.1) {
            StickPosition += 2;
            sleep(ST);
        } else if (stick < -0.1) {
            StickPosition -= 2;
            sleep(ST);
        } else if (stick == 0) {
            // Do nothing
        }

        /*if (wrist.getTargetPosition() >= limit) {
            wrist.setTargetPosition(GoodPos);
        } else if (wrist.getTargetPosition() <= 0) {
            wrist.setTargetPosition(1);
        }*/
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
