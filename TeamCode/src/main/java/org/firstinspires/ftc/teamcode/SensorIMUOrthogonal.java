package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * This OpMode demonstrates how to use the universal IMU interface for reading Yaw, Pitch, and Roll values.
 * It supports BNO055 and BHI260 IMUs. The IMU is assumed to be mounted with the name "imu".
 *
 * The sample will display the current Yaw, Pitch, and Roll of the robot.
 * - Pitch increases as the robot tilts up at the front (rotation about X).
 * - Roll increases as the robot tilts up at the left side (rotation about Y).
 * - Yaw increases as the robot rotates counter-clockwise (rotation about Z).
 *
 * Press the Y button on the gamepad (Triangle on a PS4 controller) to reset the Yaw.
 * @noinspection ALL
 */
@TeleOp(name = "Sensor: IMU Orthogonal", group = "Sensor")
@Disabled   // Comment this out to add to the OpMode list
public class SensorIMUOrthogonal extends LinearOpMode {

    // IMU sensor object
    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the IMU sensor
        imu = hardwareMap.get(IMU.class, "imu");

        // Define the hub's mounting orientation (adjust these for your specific setup)
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        // Create orientation object for the IMU based on mounting configuration
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Initialize the IMU with the specified orientation
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Wait for the start button to be pressed
        waitForStart();

        // Main loop for updating telemetry
        while (!isStopRequested()) {

            // Display hub orientation
            telemetry.addData("Hub Orientation", "Logo=%s   USB=%s", logoDirection, usbDirection);

            // Check for Yaw reset request (via the Y button on the gamepad)
            if (gamepad1.y) {
                telemetry.addData("Yaw", "Resetting");
                imu.resetYaw();
            } else {
                telemetry.addData("Yaw", "Press Y to reset");
            }

            // Retrieve rotational angles and velocities from the IMU
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            // Display the Yaw, Pitch, Roll values and their velocities
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Y)", "%.2f Deg.", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Yaw Velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
            telemetry.addData("Pitch Velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
            telemetry.addData("Roll Velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);

            // Update the telemetry data on the driver station
            telemetry.update();
        }
    }
}
