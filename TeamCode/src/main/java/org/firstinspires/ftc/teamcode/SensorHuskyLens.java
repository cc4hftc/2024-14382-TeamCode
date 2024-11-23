package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Sensor: HuskyLens", group = "Sensor")
public class SensorHuskyLens extends LinearOpMode {

    private final int READ_PERIOD = 1;

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
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        // Set initial motor directions (assuming forward as default)
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        // Set algorithm for AprilTag recognition
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();

            // Get detected blocks
            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());

                // Check if the detected block has ID 1
                if (blocks[i].id == 1) {
                    // Move motors accordingly
                    // Set motor directions: 2 forward, 2 backwards
                    // Set power for a short time to make the robot move
                    leftFrontDrive.setPower(0.5);
                    leftBackDrive.setPower(0.5);
                    rightFrontDrive.setPower(0.5);
                    rightBackDrive.setPower(0.5);
                } else {
                    // Stop the motors
                    leftFrontDrive.setPower(0);
                    leftBackDrive.setPower(0);
                    rightFrontDrive.setPower(0);
                    rightBackDrive.setPower(0);
                }
            }

            telemetry.update();
        }
    }
}
