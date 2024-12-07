package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class Other_Auto_Test_DO_NOT_TOUCH extends LinearOpMode {

    private final int READ_PERIOD = 0;  // Period for rate limiting (in seconds)

    private HuskyLens huskyLens;

    // Declare motor variables
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

    @Override
    public void runOpMode() {
        // Initialize HuskyLens and motors
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Set initial motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

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
                    telemetry.addData("x", x);
                    telemetry.addData("y", y);

                    // Check if the detected block has ID 2 and is within the x and y range
                    if (blocks[i].id == 2 && x >= 145 && x <= 175 && y >= 105 && y <= 135) {
                        // Calculate motor power based on x and y positions

                        // Set motor power based on x and y positions
                        /*leftFrontDrive.setPower(motorfront);
                        leftBackDrive.setPower(motorback);
                        rightFrontDrive.setPower(motorleft);
                        rightBackDrive.setPower(motorright);*/

                        leftFrontDrive.setPower(1);
                        leftBackDrive.setPower(1);
                        rightFrontDrive.setPower(1);
                        rightBackDrive.setPower(1);

                        //X<=MAX && X>=MIN && Y<=MAX && Y>=MIN
                    } else if (blocks[i].id == 2 && x<=145 && x>=0 && y<=105 && y>=0) {
                        leftFrontDrive.setPower(0.2);
                        leftBackDrive.setPower(0.2);
                        rightFrontDrive.setPower(0.2);
                        rightBackDrive.setPower(0.2);
                    } else if (blocks[i].id == 2 && x<=320 && x>=175 && y<=105 && y>=0) {
                        leftFrontDrive.setPower(0.3);
                        leftBackDrive.setPower(0.3);
                        rightFrontDrive.setPower(0.3);
                        rightBackDrive.setPower(0.3);
                    } else if (blocks[i].id == 2 && x<=145 && x>=0 && y<=240 && y>=135) {
                        leftFrontDrive.setPower(0.4);
                        leftBackDrive.setPower(0.4);
                        rightFrontDrive.setPower(0.4);
                        rightBackDrive.setPower(0.4);
                    } else if (blocks[i].id == 2 && x<=320 && x>=175 && y<=240 && y>=135) {
                        leftFrontDrive.setPower(0.5);
                        leftBackDrive.setPower(0.5);
                        rightFrontDrive.setPower(0.5);
                        rightBackDrive.setPower(0.5);

                    }

                    telemetry.addData("LeftFront", leftFrontDrive);
                    telemetry.addData("LeftBack", leftFrontDrive);
                    telemetry.addData("RightFront", leftFrontDrive);
                    telemetry.addData("RightBack", leftFrontDrive);

                    telemetry.update();
                }
            }
        }
    }


}