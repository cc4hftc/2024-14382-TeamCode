package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/** @noinspection ALL*/
@Autonomous
public class AutoIMU extends LinearOpMode {
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private IMU imu;

    @Override
    public void runOpMode() {
        // Initialize motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Set motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");

        // Wait for the start command
        waitForStart();

        // ------------------------------------------------------------------
        // FIRST TURN (Counterclockwise -60°)
        // ------------------------------------------------------------------
        // Get the current yaw (Z axis)
        YawPitchRollAngles currentOrientation = imu.getRobotYawPitchRollAngles();
        double currentYaw = currentOrientation.getYaw(AngleUnit.DEGREES);

        // Calculate the target yaw (current yaw - 60 degrees)
        double targetYaw = currentYaw - 60.0;

        // Normalize targetYaw to the range -180 to 180 degrees
        targetYaw = normalizeYaw(targetYaw);

        // Rotate the robot until it reaches the target yaw (counterclockwise)
        while (opModeIsActive()) {
            // Get the current yaw again
            currentOrientation = imu.getRobotYawPitchRollAngles();
            double currentYawNow = currentOrientation.getYaw(AngleUnit.DEGREES);

            // Normalize currentYawNow
            currentYawNow = normalizeYaw(currentYawNow);

            // Check if we have reached the target yaw
            if (Math.abs(currentYawNow - targetYaw) < 2.0) { // 2 degrees tolerance
                // Stop the robot once the target yaw is reached
                stopMotors();
                telemetry.addData("Status", "Target yaw reached: %.2f degrees", targetYaw);
                telemetry.update();
                break;
            }

            // Rotate counterclockwise to targetYaw
            setMotorPowers(0.3, false);  // false => counterclockwise

            // Display debug info
            telemetry.addData("Current Yaw", "%.2f", currentYawNow);
            telemetry.addData("Target Yaw", "%.2f", targetYaw);
            telemetry.update();
        }

        // ------------------------------------------------------------------
        // MOVE FORWARD 1 UNIT USING BACK MOTORS
        // ------------------------------------------------------------------
        moveAlongYAxis(1); // Adjust distance if needed
        // After moving, we want to ensure the back motors can spin freely again:
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Also ensure the front motors are in a free mode:
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Sleep a bit to ensure motors fully stop
        sleep(50);

        // ------------------------------------------------------------------
        // SECOND TURN (Clockwise +60° from the NEW yaw)
        // ------------------------------------------------------------------
        // RECHECK current yaw before second turn:
        currentOrientation = imu.getRobotYawPitchRollAngles();
        currentYaw = currentOrientation.getYaw(AngleUnit.DEGREES);
        currentYaw = normalizeYaw(currentYaw);

        // Calculate new targetYaw (clockwise from the robot's actual heading)
        targetYaw = currentYaw + 55.0;
        targetYaw = normalizeYaw(targetYaw);

        // Rotate the robot until it reaches the new target yaw (clockwise)
        while (opModeIsActive()) {
            // Get the current yaw again
            currentOrientation = imu.getRobotYawPitchRollAngles();
            double currentYawNow = currentOrientation.getYaw(AngleUnit.DEGREES);

            // Normalize currentYawNow
            currentYawNow = normalizeYaw(currentYawNow);

            // Check if we have reached the new target yaw
            if (Math.abs(currentYawNow - targetYaw) < 2.0) { // 2 degrees tolerance
                // Stop the robot once the target yaw is reached
                stopMotors();
                telemetry.addData("Status", "Target yaw reached: %.2f degrees", targetYaw);
                telemetry.update();
                break;
            }

            // Rotate clockwise to targetYaw
            setMotorPowers(0.3, true);  // true => clockwise

            // Display debug info
            telemetry.addData("Current Yaw", "%.2f", currentYawNow);
            telemetry.addData("Target Yaw", "%.2f", targetYaw);
            telemetry.update();
        }

        // Optionally stop all motors here if needed
        stopMotors();
    }

    // ----------------------------------------------------------------------
    // Helper function to set the motor powers for rotation
    // ----------------------------------------------------------------------
    private void setMotorPowers(double power, boolean clockwise) {
        if (clockwise) {
            // For clockwise rotation, reverse the power on the left motors
            leftFrontDrive.setPower(-power);
            leftBackDrive.setPower(-power);
            rightFrontDrive.setPower(power);
            rightBackDrive.setPower(power);
        } else {
            // For counterclockwise rotation (default behavior)
            leftFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightFrontDrive.setPower(-power);
            rightBackDrive.setPower(-power);
        }
    }

    // ----------------------------------------------------------------------
    // Helper function to stop all motors
    // ----------------------------------------------------------------------
    private void stopMotors() {
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    // ----------------------------------------------------------------------
    // Normalize yaw to the range -180 to 180 degrees
    // ----------------------------------------------------------------------
    private double normalizeYaw(double yaw) {
        while (yaw <= -180.0) {
            yaw += 360.0;
        }
        while (yaw > 180.0) {
            yaw -= 360.0;
        }
        return yaw;
    }

    // ----------------------------------------------------------------------
    // Move the robot along the Y-axis using only the back motors (leftBack, rightBack)
    // ----------------------------------------------------------------------
    private void moveAlongYAxis(double distance) {
        // Calculate the number of encoder counts required for the given distance
        // (Adjust based on your robot’s wheel/circumference/encoder specs)
        int targetPosition = (int) (distance * 1440); // Example: 1440 ticks per revolution

        // Reset encoders for the back motors
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set target positions (positive => forward, negative => backward)
        leftBackDrive.setTargetPosition(targetPosition);
        rightBackDrive.setTargetPosition(targetPosition);

        // Set motor powers to move
        setBackMotorPowers(0.5);  // Adjust speed as needed

        // Wait until the robot reaches the target position
        while (opModeIsActive() && leftBackDrive.isBusy() && rightBackDrive.isBusy()) {
            telemetry.addData("Moving", "Distance: %.2f units", distance);
            telemetry.update();
        }

        // Stop the back motors after movement
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    // ----------------------------------------------------------------------
    // Helper function to set the motor powers for the back motors only
    // ----------------------------------------------------------------------
    private void setBackMotorPowers(double power) {
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
    }
}
