package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Combined Autonomous that:
 * 1) Uses IMU to rotate/move.
 * 2) Activates HuskyLens color recognition.
 * 3) Aligns with the detected color and operates the claw.
 */
@Autonomous(name = "AutoIMU_HuskyLens_Combined")
public class AutoIMU_HuskyLens_Combined extends LinearOpMode {

    // -------------------------------
    // Hardware: Drive train
    // -------------------------------
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

    // -------------------------------
    // Hardware: IMU
    // -------------------------------
    private IMU imu;

    // -------------------------------
    // Hardware: HuskyLens
    // -------------------------------
    private HuskyLens huskyLens;

    // -------------------------------
    // Hardware: Arm & Servos
    // -------------------------------
    private DcMotorEx armMotor = null;
    private DcMotorEx otherArmMotor = null;
    private Servo claw = null;
    private Servo other_claw = null;
    private Servo wrist = null;

    // -------------------------------
    // PID / Arm Control
    // -------------------------------
    private double otherTargetPosition;
    private double targetPosition;
    private double kP = 0.01; //0.001
    private double kI = 0.00;
    private double kD = 0.000;
    private double kF = 0.01;
    private double integral = 0;
    private double lastError = 0;
    private double otherlastError = 0;
    private ElapsedTime timer = new ElapsedTime();
    private boolean pidControlActive = false;

    // -------------------------------
    // HuskyLens alignment parameters
    // -------------------------------
    private final int MIN_TARGET_X = 145;
    private final int MAX_TARGET_X = 195;
    private final int MIN_TARGET_Y = 120;
    private final int MAX_TARGET_Y = 125;

    // -------------------------------
    // Arm limit example
    // -------------------------------
    //int limit = 380;

    // -------------------------------
    // For rate limiting HuskyLens read
    // -------------------------------
    private final int READ_PERIOD = 0;  // in seconds

    @Override
    public void runOpMode() {
        // -------------------------------------------------------
        // 1) Initialize all hardware
        // -------------------------------------------------------
        // Motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Drive motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");

        // HuskyLens
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        // Arm & Servos
        armMotor      = hardwareMap.get(DcMotorEx.class, "arm_motor");
        otherArmMotor = hardwareMap.get(DcMotorEx.class, "arm_motor2");
        claw          = hardwareMap.get(Servo.class, "clawServo");
        other_claw    = hardwareMap.get(Servo.class, "other_clawServo");
        wrist         = hardwareMap.get(Servo.class, "wristServo");

        // Arm motor directions
        //armMotor.setDirection(DcMotorEx.Direction.FORWARD);
        //otherArmMotor.setDirection(DcMotorEx.Direction.REVERSE);
        claw.setDirection(Servo.Direction.FORWARD);
        other_claw.setDirection(Servo.Direction.REVERSE);

        // Reset & run encoders
        otherArmMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        otherArmMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Initialize claw positions (example)
        claw.setPosition(0.1775);
        other_claw.setPosition(0.1775);

        // Initialize HuskyLens
        if (!huskyLens.knock()) {
            telemetry.addData("HuskyLens", "Communication problem with " + huskyLens.getDeviceName());
        } else {
            huskyLens.initialize();
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
            telemetry.addData("HuskyLens", "Initialized - press start.");
        }
        telemetry.update();

        // Wait for the start command
        waitForStart();

        // -------------------------------------------------------
        // 2) IMU-based motion: First turn ~-60°, move forward, second turn ~+55°
        // -------------------------------------------------------
        // FIRST TURN: -60 degrees (counterclockwise)
        YawPitchRollAngles currentOrientation = imu.getRobotYawPitchRollAngles();
        double currentYaw = currentOrientation.getYaw(AngleUnit.DEGREES);
        double targetYaw  = normalizeYaw(currentYaw - 60.0);
        sleep(500);
        while (opModeIsActive()) {
            currentOrientation = imu.getRobotYawPitchRollAngles();
            double currentYawNow = normalizeYaw(currentOrientation.getYaw(AngleUnit.DEGREES));

            if (Math.abs(currentYawNow - targetYaw) < 2.0) {
                stopMotors();
                telemetry.addData("Status", "Reached first target yaw: %.2f", targetYaw);
                telemetry.update();
                break;
            }
            // Rotate CCW
            setMotorPowers(0.3, false);
            telemetry.addData("Current Yaw", "%.2f", currentYawNow);
            telemetry.addData("Target Yaw", "%.2f", targetYaw);
            telemetry.update();
        }

        // MOVE FORWARD 1.3 unit using back motors
        moveAlongYAxis(1.3);
        // Return back motors to run_using_encoder
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(50);

        // SECOND TURN: +68 degrees (clockwise from new yaw)
        currentOrientation = imu.getRobotYawPitchRollAngles();
        currentYaw = normalizeYaw(currentOrientation.getYaw(AngleUnit.DEGREES));
        targetYaw  = normalizeYaw(currentYaw + 65.0);
        while (opModeIsActive()) {
            currentOrientation = imu.getRobotYawPitchRollAngles();
            double currentYawNow = normalizeYaw(currentOrientation.getYaw(AngleUnit.DEGREES));

            if (Math.abs(currentYawNow - targetYaw) < 2.0) {
                stopMotors();
                telemetry.addData("Status", "Reached second target yaw: %.2f", targetYaw);
                telemetry.update();
                break;
            }
            // Rotate CW
            setMotorPowers(0.3, true);
            telemetry.addData("Current Yaw", "%.2f", currentYawNow);
            telemetry.addData("Target Yaw", "%.2f", targetYaw);
            telemetry.update();
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        }

        stopMotors();

        // -------------------------------------------------------
        // 3) Activate HuskyLens color recognition & align with color
        //    Then operate the claw.
        // -------------------------------------------------------
        // Example: Move the arms up to a 'limit' position before alignment
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        timer.reset();

        // Move arms to initial position
        /*armMotor.setTargetPosition(limit);
        otherArmMotor.setTargetPosition(limit);
        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        otherArmMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);*/
        // armMotor.setPower(0.25);       // If needed
        // otherArmMotor.setPower(0.25); // If needed

        // Wait for arms to reach target (optional)
        /*
        while (opModeIsActive() && (armMotor.isBusy() || otherArmMotor.isBusy())) {
            telemetry.addData("Arm1 Target/Pos", "%d/%d", armMotor.getTargetPosition(), armMotor.getCurrentPosition());
            telemetry.addData("Arm2 Target/Pos", "%d/%d", otherArmMotor.getTargetPosition(), otherArmMotor.getCurrentPosition());
            telemetry.update();
        }
        */

        // Once the arms are at the limit, hold them with PID
        /*resetPID();
        targetPosition = armMotor.getCurrentPosition();
        otherTargetPosition = otherArmMotor.getCurrentPosition();
        pidControlActive = true; // We can keep them locked in place if desired*/

        // Rate-limited HuskyLens loop
        double lastTime = getRuntime();
        while (opModeIsActive()) {
            // If enough time has passed, read from HuskyLens
            if (getRuntime() - lastTime >= READ_PERIOD) {
                lastTime = getRuntime();

                HuskyLens.Block[] blocks = huskyLens.blocks();
                telemetry.addData("Block count", blocks.length);

                // Stop any leftover movement from the drive train
                stopMotors();

                // Check each block (color) recognized
                for (HuskyLens.Block block : blocks) {
                    telemetry.addData("Block", block.toString());
                    int x = block.x;
                    int y = block.y;

                    // Example: we only move if the block has ID == 2
                    if (block.id == 2) {
                        // X alignment
                        if (x < MIN_TARGET_X) {
                            // Robot turns left
                            leftFrontDrive.setPower(-0.25);                     // Back
                            leftBackDrive.setPower(-0.25);                      // Left
                            rightFrontDrive.setPower(0.25);                     // Front
                            rightBackDrive.setPower(0.25);                      // Right
                        } else if (x > MAX_TARGET_X) {
                            // Robot turns right
                            leftFrontDrive.setPower(0.25);                      // Back
                            leftBackDrive.setPower(0.25);                       // Left
                            rightFrontDrive.setPower(-0.25);                    // Front
                            rightBackDrive.setPower(-0.25);                     // Right
                        } else {
                            // Stop turning
                            stopMotors();
                        }

                        // Y alignment
                        if (y < MIN_TARGET_Y) {
                            // Move forward
                            leftBackDrive.setPower(0.25);
                            rightBackDrive.setPower(0.25);
                        } else if (y > MAX_TARGET_Y) {
                            // Move backward
                            leftBackDrive.setPower(-0.25);
                            rightBackDrive.setPower(-0.25);
                        } else {
                            stopMotors();
                        }

                        // Once we're in the target zone, do the claw action
                        if (x >= MIN_TARGET_X && x <= MAX_TARGET_X && y >= MIN_TARGET_Y && y <= MAX_TARGET_Y) {
                            sleep(500);
                            stopMotors();
                            //pidControlActive = false; // If you want to turn off the arm-holding PID

                            // Switch to some other algorithm (optional)
                            huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
                            sleep(250);

                            // Example wrist/claw sequence
                            wrist.setPosition(0.75);
                            sleep(250);
                            claw.setPosition(0.065);
                            other_claw.setPosition(0.075);
                            sleep(1500);

                            // Return the claw to "closed" or "open" as needed
                            claw.setPosition(0.1775);
                            other_claw.setPosition(0.1775);
                            sleep(750);

                            wrist.setPosition(0);
                            sleep(500);
                            thirdTurn();
                        }
                    }
                    /*if (block.id == 3) {
                        // X alignment
                        if (x < MIN_TARGET_X) {
                            // Robot turns left
                            leftFrontDrive.setPower(-0.25);                     // Back
                            //leftBackDrive.setPower(-0.25);                      // Left
                            rightFrontDrive.setPower(0.25);                     // Front
                            //rightBackDrive.setPower(0.25);                      // Right
                        } else if (x > MAX_TARGET_X) {
                            // Robot turns right
                            leftFrontDrive.setPower(0.25);                      // Back
                            //leftBackDrive.setPower(0.25);                       // Left
                            rightFrontDrive.setPower(-0.25);                    // Front
                            //rightBackDrive.setPower(-0.25);                     // Right
                        } else {
                            // Stop turning
                            //stopMotors();
                        }
                    }*/
                }
                telemetry.update();
            }

            // If PID is active, hold the arm position
            if (pidControlActive) {
                //pidControl();
            }
        }
    }

    // ------------------------------------------------------------------------
    // HELPER: Set motor powers for rotation
    // ------------------------------------------------------------------------
    private void setMotorPowers(double power, boolean clockwise) {
        if (clockwise) {
            // Clockwise: left motors negative, right motors positive
            leftFrontDrive.setPower(-power);
            leftBackDrive.setPower(-power);
            rightFrontDrive.setPower(power);
            rightBackDrive.setPower(power);
        } else {
            // Counterclockwise
            leftFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightFrontDrive.setPower(-power);
            rightBackDrive.setPower(-power);
        }
    }

    private void thirdTurn() {
        // THIRD TURN: +81 degrees (clockwise from new yaw)
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        YawPitchRollAngles currentOrientation = imu.getRobotYawPitchRollAngles();
        double currentYaw = normalizeYaw(currentOrientation.getYaw(AngleUnit.DEGREES));
        double third = (70 - currentYaw);
        double targetYaw  = normalizeYaw(third);
        while (opModeIsActive()) {
            currentOrientation = imu.getRobotYawPitchRollAngles();
            double currentYawNow = normalizeYaw(currentOrientation.getYaw(AngleUnit.DEGREES));

            if (Math.abs(currentYawNow - targetYaw) < 2.0) {
                stopMotors();
                telemetry.addData("Status", "Reached second target yaw: %.2f", targetYaw);
                telemetry.update();
                break;
            }
            // Rotate CW
            setMotorPowers(0.3, true);
            telemetry.addData("Current Yaw", "%.2f", currentYawNow);
            telemetry.addData("Target Yaw", "%.2f", targetYaw);
            telemetry.update();
        }
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        sleep(850);
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        moveAlongYAxis(2);
        sleep(500);
        AprilTag();
        sleep(500);
        moveAlongYAxis(0.5);
        sleep(500);
        AprilTag();
        sleep(500);
        moveAlongYAxis(0.6);
    }

    private void AprilTag() {
        HuskyLens.Block[] blocks = huskyLens.blocks();
        telemetry.addData("Block count", blocks.length);

        for (HuskyLens.Block block : blocks) {
            telemetry.addData("Block", block.toString());
            int x = block.x;
            int y = block.y;

            if (block.id == 3) {
                // X alignment
                if (x < MIN_TARGET_X) {
                    // Robot turns left
                    leftFrontDrive.setPower(-0.25);                     // Back
                    //leftBackDrive.setPower(-0.25);                      // Left
                    rightFrontDrive.setPower(0.25);                     // Front
                    //rightBackDrive.setPower(0.25);                      // Right
                } else if (x > MAX_TARGET_X) {
                    // Robot turns right
                    leftFrontDrive.setPower(0.25);                      // Back
                    //leftBackDrive.setPower(0.25);                       // Left
                    rightFrontDrive.setPower(-0.25);                    // Front
                    //rightBackDrive.setPower(-0.25);                     // Right
                } else {
                    // Stop turning
                    //stopMotors();
                }
            }
        }
    }

    // ------------------------------------------------------------------------
    // HELPER: Stop all motors
    // ------------------------------------------------------------------------
    private void stopMotors() {
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    // ------------------------------------------------------------------------
    // HELPER: Normalize yaw to [-180, +180]
    // ------------------------------------------------------------------------
    private double normalizeYaw(double yaw) {
        while (yaw <= -180.0) {
            yaw += 360.0;
        }
        while (yaw > 180.0) {
            yaw -= 360.0;
        }
        return yaw;
    }

    // ------------------------------------------------------------------------
    // HELPER: Move along Y-axis using back motors
    // ------------------------------------------------------------------------
    private void moveAlongYAxis(double distance) {
        // Example: 1440 ticks per revolution
        int targetPosition = (int) (distance * 1440);

        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftBackDrive.setTargetPosition(targetPosition);
        rightBackDrive.setTargetPosition(targetPosition);

        setBackMotorPowers(0.5);

        while (opModeIsActive() && leftBackDrive.isBusy() && rightBackDrive.isBusy()) {
            telemetry.addData("Moving forward", "Distance: %.2f units", distance);
            telemetry.update();
        }

        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    // ------------------------------------------------------------------------
    // HELPER: Set back motors only
    // ------------------------------------------------------------------------
    private void setBackMotorPowers(double power) {
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
    }

    // ------------------------------------------------------------------------
    // HELPER: Simple PID control for holding arm position
    // ------------------------------------------------------------------------
    private void pidControl() {
        double otherCurrentPosition = otherArmMotor.getCurrentPosition();
        double currentPosition      = armMotor.getCurrentPosition();

        double otherError = otherTargetPosition - otherCurrentPosition;
        double error      = targetPosition      - currentPosition;
        double deltaTime  = timer.seconds();

        integral += otherError * deltaTime;
        double derivative = (otherError - lastError) / deltaTime;

        double power      = kP * error      + kI * integral + kD * derivative + kF;
        double otherPower = kP * otherError + kI * integral + kD * derivative + kF;

        armMotor.setPower(power);
        otherArmMotor.setPower(otherPower);

        lastError = error;
        otherlastError = otherError;
        timer.reset();
    }

    // ------------------------------------------------------------------------
    // HELPER: Reset PID accumulators
    // ------------------------------------------------------------------------
    private void resetPID() {
        integral = 0;
        lastError = 0;
        otherlastError = 0;
        timer.reset();
    }
}
