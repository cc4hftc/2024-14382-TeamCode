package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class ArmControlTeleOp extends LinearOpMode {
    DcMotor armMotor = null;
    DcMotor other_ArmMotor = null;

    @Override
    public void runOpMode() {
        // Initialize both motors
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        other_ArmMotor = hardwareMap.get(DcMotor.class, "arm_motor2");
        // Optionally set motor direction if needed
        armMotor.setDirection(DcMotor.Direction.FORWARD); // Change if necessary
        other_ArmMotor.setDirection(DcMotor.Direction.REVERSE); // Change if needed

        waitForStart();

        while (opModeIsActive()) {
            // Control the arm with the triggers
            double armPower = gamepad2.right_trigger - gamepad2.left_trigger;

            // Optionally, control both motors together
            if (gamepad2.a) {
                armMotor.setPower(0.5); // Move arm up
                other_ArmMotor.setPower(0.5); // Move other arm up
            } else if (gamepad2.b) {
                armMotor.setPower(-0.5); // Move arm down
                other_ArmMotor.setPower(-0.5); // Move other arm down
            } else {
                armMotor.setPower(0); // Stop the arm
                other_ArmMotor.setPower(0); // Stop the other arm
            }

            // Optionally, control both motors with the trigger-based power
            // Uncomment the following lines if you want both motors to react to the triggers equally
            // armMotor.setPower(armPower);
            // other_ArmMotor.setPower(armPower);

            // Add a small delay to prevent rapid updates (optional)
            sleep(50);
        }
    }
}
