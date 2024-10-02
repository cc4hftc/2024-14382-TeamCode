package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class Auto extends LinearOpMode {
    DcMotor leftFrontDrive;
    DcMotor leftBackDrive;
    DcMotor rightFrontDrive;
    DcMotor rightBackDrive;
    DistanceSensor distanceSensor;

    private static final double TURN_SPEED_FACTOR = 0.5;
    private static final double SAFE_DISTANCE_CM = 30.0; // Adjust as necessary
    private static final double MOVE_SPEED = 0.5; // Move speed

    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");

        // Optionally set motor directions if needed
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Initialize TensorFlow (if using TensorFlow for vision)
        // TensorFlow initialization code goes here

        waitForStart();

        while (opModeIsActive()) {
            double distance = distanceSensor.getDistance(DistanceUnit.CM);

            if (distance < SAFE_DISTANCE_CM) {
                // Obstacle detected, stop and turn
                stopMotors();
                turnLeft();
            } else {
                // No obstacle detected, move forward
                moveForward();
            }

            // Vision processing code goes here (if applicable)
            // You can use TensorFlow to detect specific objects and adjust behavior
        }

        // Stop all motors when finished
        stopMotors();
    }

    private void moveForward() {
        leftFrontDrive.setPower(MOVE_SPEED);
        leftBackDrive.setPower(MOVE_SPEED);
        rightFrontDrive.setPower(MOVE_SPEED);
        rightBackDrive.setPower(MOVE_SPEED);
    }

    private void stopMotors() {
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private void turnLeft() {
        leftFrontDrive.setPower(-MOVE_SPEED);
        leftBackDrive.setPower(-MOVE_SPEED);
        rightFrontDrive.setPower(MOVE_SPEED);
        rightBackDrive.setPower(MOVE_SPEED);

        // Add a delay or condition to stop turning
        sleep(500); // Turn for half a second, adjust as needed
        stopMotors();
    }
}
