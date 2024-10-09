package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class ArmControlTeleOp extends LinearOpMode {
    DcMotor armMotor = null;
    DcMotor other_ArmMotor = null;

    // Encoder positions corresponding to 0 and 180 degrees
    private static final int ARM_MIN_POSITION = -1000;  // Adjust this based on actual arm configuration
    private static final int ARM_MAX_POSITION = 1000;  // Adjust this based on actual arm configuration

    @Override
    public void runOpMode() {
        // Initialize both motors
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        other_ArmMotor = hardwareMap.get(DcMotor.class, "arm_motor2");

        // Optionally set motor direction if needed
        armMotor.setDirection(DcMotor.Direction.FORWARD); // Change if necessary
        other_ArmMotor.setDirection(DcMotor.Direction.REVERSE); // Change if needed

        // Set the motors to use encoders
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        other_ArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            // Control the arm with the triggers
            double armPower = gamepad2.right_trigger - gamepad2.left_trigger;

            // Get the current position of the arm motors
            int currentPosition = armMotor.getCurrentPosition();

            // Move the arm up (increase position) if the power is positive, but respect the limits
            if (armPower > 0 && currentPosition < ARM_MAX_POSITION) {
                armMotor.setPower(armPower);
                other_ArmMotor.setPower(armPower);
            }
            // Move the arm down (decrease position) if the power is negative, but respect the limits
            else if (armPower < 0 && currentPosition > ARM_MIN_POSITION) {
                armMotor.setPower(armPower);
                other_ArmMotor.setPower(armPower);
            }
            // Stop the arm if it reaches the boundaries
            else {
                armMotor.setPower(0);
                other_ArmMotor.setPower(0);
            }

            // Optionally, control both motors together with a button press (e.g., A and B)
            if (gamepad2.a) {
                // Ensure we don't exceed the max position
                if (currentPosition < ARM_MAX_POSITION) {
                    armMotor.setPower(10); // Move arm up
                    other_ArmMotor.setPower(10); // Move other arm up
                }
            } else if (gamepad2.b) {
                // Ensure we don't go below the min position
                if (currentPosition > ARM_MIN_POSITION) {
                    armMotor.setPower(-10); // Move arm down
                    other_ArmMotor.setPower(-10); // Move other arm down
                }
            } else {
                armMotor.setPower(0); // Stop the arm
                other_ArmMotor.setPower(0); // Stop the other arm
            }

            // Add a small delay to prevent rapid updates (optional)
            sleep(50);
        }
    }
}
