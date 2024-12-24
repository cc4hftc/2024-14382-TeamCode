package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/** @noinspection ALL*/
@Autonomous
public class Other_Auto_Test_DO_NOT_TOUCH_PIDF extends LinearOpMode {

    private final int READ_PERIOD = 0;  // Period for rate limiting (in seconds)

    private HuskyLens huskyLens;

    // Declare motor variables
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    DcMotorEx armMotor = null;
    DcMotorEx otherArmMotor = null;
    Servo claw = null; // Claw servo reference
    Servo other_claw = null; // Claw servo reference
    Servo wrist = null;

    // PID control constants for the arm
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

    // Define target area boundaries
    private final int MIN_TARGET_X = 135;
    private final int MAX_TARGET_X = 205;
    private final int MIN_TARGET_Y = 115;
    private final int MAX_TARGET_Y = 130;

    // Flag to control PID activation
    private boolean pidControlActive = false;

    int limit = 380;

    @Override
    public void runOpMode() {
        // Initialize HuskyLens and motors
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        armMotor = hardwareMap.get(DcMotorEx.class, "arm_motor");               // Port 2
        otherArmMotor = hardwareMap.get(DcMotorEx.class, "arm_motor2");         // Port 1
        claw = hardwareMap.get(Servo.class, "clawServo");                       // Port 1
        other_claw = hardwareMap.get(Servo.class, "other_clawServo");
        wrist = hardwareMap.get(Servo.class, "wristServo");

        // Set initial motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);                         // Back
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);                          // Left
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);                        // Front
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);                         // Right
        armMotor.setDirection(DcMotorEx.Direction.FORWARD);
        otherArmMotor.setDirection(DcMotorEx.Direction.REVERSE);
        claw.setDirection(Servo.Direction.FORWARD);
        other_claw.setDirection(Servo.Direction.REVERSE);

        otherArmMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        otherArmMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        claw.setPosition(0.1775);
        other_claw.setPosition(0.1775);
        //Move before starting to try to remove slack
        //armMotor.setPower(0.0025);
        //otherArmMotor.setPower(0.0025);
        // Initialize HuskyLens
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
            huskyLens.initialize();
        }

        // Set algorithm for object recognition
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        telemetry.update();
        waitForStart();
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        timer.reset();

        // Set the initial arm positions (move armMotor to 76 and otherArmMotor to 88)
        armMotor.setTargetPosition(limit);
        otherArmMotor.setTargetPosition(limit);
        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        otherArmMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Set initial power to move the arm to target positions
        //armMotor.setPower(0.25);  // Adjust power as needed for the arm to reach the target
        //otherArmMotor.setPower(0.25);  // Adjust power as needed for the arm to reach the target

        // Wait until the arm motors reach the target position
        /*while (opModeIsActive() && (armMotor.isBusy() || otherArmMotor.isBusy())) {
            telemetry.addData("Arm Target", "Arm1: %d, Arm2: %d", armMotor.getTargetPosition(), otherArmMotor.getTargetPosition());
            telemetry.addData("Arm Position", "Arm1: %d, Arm2: %d", armMotor.getCurrentPosition(), otherArmMotor.getCurrentPosition());
            telemetry.addData("Tick", otherArmMotor.getCurrentPosition());
            telemetry.update();
        }*/

        // Once the arm reaches the target positions, apply PID control to hold the position
        resetPID();

        // Set target positions to current positions to maintain them
        targetPosition = armMotor.getCurrentPosition();
        otherTargetPosition = otherArmMotor.getCurrentPosition();

        // Rate-limited HuskyLens data reading and motor control loop
        double lastTime = getRuntime();
        while (opModeIsActive()) {
            // Check if enough time has passed for the next read cycle
            if (getRuntime() - lastTime >= READ_PERIOD) {
                lastTime = getRuntime();  // Reset the timer

                // Get detected blocks from HuskyLens
                HuskyLens.Block[] blocks = huskyLens.blocks();
                telemetry.addData("Block count", blocks.length);

                leftFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightFrontDrive.setPower(0);
                rightBackDrive.setPower(0);

                for (int i = 0; i < blocks.length; i++) {
                    telemetry.addData("Block", blocks[i].toString());
                    int x = blocks[i].x;
                    int y = blocks[i].y;

                    // Calculate motor power adjustments based on object position
                    double POSX = 0.25 - (x * 0.002);
                    double NEGX = -(0.25 - (x * 0.002));
                    double POSY = 1 - (y * 0.005);
                    double NEGY = -(1 - (y * 0.0064));

                    double tick = armMotor.getCurrentPosition();

                    /*if (tick < limit) {
                        armMotor.setPower(0.1);
                        otherArmMotor.setPower(0.1);
                        pidControlActive = false;
                    } else if (tick > limit) {
                        armMotor.setPower(0);
                        otherArmMotor.setPower(0);
                        pidControlActive = true;
                    }*/

                    // Control robot movement based on detected object position
                    if (blocks[i].id == 2) {
                        // Move robot towards target based on X and Y positions
                        if (x < MIN_TARGET_X) {
                            leftFrontDrive.setPower(NEGX);
                            leftBackDrive.setPower(NEGX);
                            rightFrontDrive.setPower(POSX);
                            rightBackDrive.setPower(POSX);
                        } else if (x > MAX_TARGET_X) {
                            leftFrontDrive.setPower(NEGX);
                            leftBackDrive.setPower(NEGX);
                            rightFrontDrive.setPower(POSX);
                            rightBackDrive.setPower(POSX);
                        } else {
                            leftFrontDrive.setPower(0);
                            leftBackDrive.setPower(0);
                            rightFrontDrive.setPower(0);
                            rightBackDrive.setPower(0);
                        }

                        if (y < MIN_TARGET_Y) {
                            leftBackDrive.setPower(POSY);
                            rightBackDrive.setPower(POSY);
                        } else if (y > MAX_TARGET_Y) {
                            leftBackDrive.setPower(NEGY);
                            rightBackDrive.setPower(NEGY);
                        }

                        // Perform action once object is in the target zone
                        if (x >= MIN_TARGET_X && x <= MAX_TARGET_X && y >= MIN_TARGET_Y && y <= MAX_TARGET_Y) {
                            sleep(500);
                            pidControlActive = false;
                            huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
                            sleep(250);
                            wrist.setPosition(0.85);
                            sleep(250);
                            claw.setPosition(0.075);
                            other_claw.setPosition(0.075);
                            sleep(1500);
                            claw.setPosition(0.1775);
                            other_claw.setPosition(0.1775);
                            sleep(750);
                            wrist.setPosition(0);
                        }
                    }
                }
            }

            // PID control for arm motors if active
            if (pidControlActive) {
                pidControl();  // Hold the arm in position using PIDF
            }
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

        // Apply mirrored power to both motors to hold position
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
    }}
