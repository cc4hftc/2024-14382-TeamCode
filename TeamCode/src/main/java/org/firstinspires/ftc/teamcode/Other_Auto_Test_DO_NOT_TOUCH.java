package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

/** @noinspection ALL */
@Autonomous
public class Other_Auto_Test_DO_NOT_TOUCH extends LinearOpMode {

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
    // Define target area boundaries
    private final int MIN_TARGET_X = 135;
    private final int MAX_TARGET_X = 205;
    private final int MIN_TARGET_Y = 130;
    private final int MAX_TARGET_Y = 160;

    /** @noinspection ForLoopReplaceableByForEach*/
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



        // Communicate with the HuskyLens sensor
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        // Set algorithm for object recognition
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        telemetry.update();
        waitForStart();

        // Use a timer to manage rate limiting (every READ_PERIOD seconds)
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
                    double POSX = 0.25 - (x * 0.002);
                    double NEGX = -(0.25 - (x * 0.002));
                    double POSX1 = 0.25 - (320-x)*0.002;
                    double NEGX1 = -(0.25 - (320-x)*0.002);

                    double POSY = 1 - (y * 0.005);
                    double NEGY = -(1 - (y * 0.0064));
                    double POSY1 = 1 - (240-y)*0.0072;
                    double NEGY1 = -(1 - (240-y)*0.0072);
                    telemetry.addData("x", x);
                    telemetry.addData("y", y);

                    // Check if the detected block has ID 2 and is within the x and y range
                    if (blocks[i].id == 2) {

                        if (blocks[i].id == 2 && x < MIN_TARGET_X) {
                            leftFrontDrive.setPower(NEGX);
                            leftBackDrive.setPower(NEGX);
                            rightFrontDrive.setPower(POSX);
                            rightBackDrive.setPower(POSX);
                        }

                        if (x > MIN_TARGET_X  && x < MAX_TARGET_X) {
                            leftFrontDrive.setPower(0);
                            //leftBackDrive.setPower(0);
                            rightFrontDrive.setPower(0);
                            //rightBackDrive.setPower(0);
                            if (y < MIN_TARGET_Y) {
                                leftBackDrive.setPower(POSY);
                                rightBackDrive.setPower(POSY);
                            }
                            if (y > MAX_TARGET_Y) {
                                leftBackDrive.setPower(NEGY1);
                                rightBackDrive.setPower(NEGY1);
                            }
                        }

                        if (blocks[i].id == 2 && x > MAX_TARGET_X) {
                            leftFrontDrive.setPower(POSX1);
                            leftBackDrive.setPower(POSX1);
                            rightFrontDrive.setPower(NEGX1);
                            rightBackDrive.setPower(NEGX1);
                        }
                        if (x >= MIN_TARGET_X && x <= MAX_TARGET_X && y >= MIN_TARGET_Y && y <= MAX_TARGET_Y) {
                            sleep(500);
                            huskyLens.close();
                            wrist.setPosition(0.1);
                            claw.setPosition(0.075);
                            other_claw.setPosition(0.075);
                            sleep(500);
                            claw.setPosition(0.1775);
                            other_claw.setPosition(0.1775);
                            sleep(250);
                            wrist.setPosition(0);
                        }

                        // Calculate motor powers based on x and y positions
                        /*double motorPowerX = (x - (MIN_TARGET_X + MAX_TARGET_X) / 2) / ((MAX_TARGET_X - MIN_TARGET_X) / 2);
                        double motorPowerY = (y - (MIN_TARGET_Y + MAX_TARGET_Y) / 2) / ((MAX_TARGET_Y - MIN_TARGET_Y) / 2);

                        // Adjust motor powers based on X and Y offsets
                        double leftFrontPower = motorPowerY - motorPowerX; // Move forward/backward and rotate
                        double rightFrontPower = motorPowerY + motorPowerX; // Move forward/backward and rotate
                        double leftBackPower = motorPowerY + motorPowerX;  // Move forward/backward and rotate
                        double rightBackPower = motorPowerY - motorPowerX; // Move forward/backward and rotate

                        // Scale power to limit the max motor power to 0.5
                        leftFrontPower = Math.max(Math.min(leftFrontPower, 0.5), -0.5);
                        rightFrontPower = Math.max(Math.min(rightFrontPower, 0.5), -0.5);
                        leftBackPower = Math.max(Math.min(leftBackPower, 0.5), -0.5);
                        rightBackPower = Math.max(Math.min(rightBackPower, 0.5), -0.5);

                        // Set the motor powers to try and reach the target position
                        leftFrontDrive.setPower(leftFrontPower);
                        leftBackDrive.setPower(leftBackPower);
                        rightFrontDrive.setPower(rightFrontPower);
                        rightBackDrive.setPower(rightBackPower);*/
                    }
                }

                telemetry.update();
            }
        }
    }
}
