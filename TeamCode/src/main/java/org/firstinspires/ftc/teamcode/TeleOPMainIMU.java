package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
public class TeleOPMainIMU extends LinearOpMode {

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

    // IMU sensor
    IMU imu;
    boolean orientationIsValid = true;

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
    private static final double TURN_SPEED_FACTOR = 0.45;
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

        // Initialize IMU sensor
        imu = hardwareMap.get(IMU.class, "imu");

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
            // Read IMU data
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

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
            double armPower = gamepad2.left_trigger - gamepad2.right_trigger;

            if (rightTriggerEnabled && armPower > 0.1) {
                armMotor.setPower(armPower / 12);
                otherArmMotor.setPower(armPower / 12);
                otherTargetPosition = otherArmMotor.getCurrentPosition();
                targetPosition = armMotor.getCurrentPosition();
                resetPID();
            } else if (leftTriggerEnabled && armPower < -0.1) {
                armMotor.setPower(armPower / 12);
                otherArmMotor.setPower(armPower / 12);
                otherTargetPosition = otherArmMotor.getCurrentPosition();
                targetPosition = armMotor.getCurrentPosition();
                resetPID();
            } else {
                pidControl();
            }

            double tick = otherArmMotor.getCurrentPosition();

            if (gamepad1.a) {
                winch.setPower(0.5);
            } else if (gamepad1.b) {
                winch.setPower(-0.5);
            } else {
                winch.setPower(0);
            }

            if (tick >= 375) {
                rightTriggerEnabled = false;
            } else if (tick >= 1 && tick <= 374) {
                rightTriggerEnabled = true;
                leftTriggerEnabled = true;
            } else if (tick <= 0) {
                leftTriggerEnabled = false;
                rightTriggerEnabled = true;
            }

            armMotor.setTargetPosition(otherArmMotor.getTargetPosition());

            // Wrist control
            if (!gamepad1.x) {
                if ((newWrist > 0.7 && newWrist < 0.9) && (gamepad1.left_stick_y != 0 || gamepad1.right_stick_x != 0)) {
                    newWrist = 0.6;
                }
            }

            // Set the wrist target position
            newWrist += (gamepad2.right_stick_y / 150);
            newWrist = Math.max(0, Math.min(0.85, newWrist));
            wrist.setPosition(newWrist);

            // Claw control
            if (gamepad2.right_bumper) {  // Close
                newClaw = 0.1775;
            } else if (gamepad2.left_bumper) {  // Open
                newClaw = 0.075;
            }

            claw.setPosition(newClaw);
            other_claw.setPosition(newClaw);

            // Debug IMU data
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Y)", "%.2f Deg.", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Yaw Velocity (Z)", "%.2f Deg/Sec", angularVelocity.zRotationRate);
            telemetry.addData("Pitch Velocity (X)", "%.2f Deg/Sec", angularVelocity.xRotationRate);
            telemetry.addData("Roll Velocity (Y)", "%.2f Deg/Sec", angularVelocity.yRotationRate);

            // Other debug info
            telemetry.addData("Arm Data", "Other Arm Pos: " + otherArmMotor.getCurrentPosition());
            telemetry.addData("Arm Data", "Arm Pos: " + armMotor.getCurrentPosition());

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
