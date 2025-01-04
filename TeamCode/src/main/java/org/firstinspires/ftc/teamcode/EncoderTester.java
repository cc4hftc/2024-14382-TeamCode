package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class EncoderTester extends LinearOpMode {
    // Declare the motor
    private DcMotor motor;

    @Override
    public void runOpMode() {
        // Initialize the hardware (motor)
        motor = hardwareMap.get(DcMotor.class, "motor");  // Ensure "motor" is the name of your motor in the configuration
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset the encoder
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Start

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            // Display the encoder value on the Driver Station
            telemetry.addData("Encoder Value", motor.getCurrentPosition());
            telemetry.update();

            // Optionally, you can set the motor to run for testing purposes
            motor.setPower(0.5);  // Run the motor at half power

            // You can also stop the motor after a set amount of time
            if (getRuntime() > 10) {
                motor.setPower(0);  // Stop the motor after 10 seconds
            }
        }
    }
}
