package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class ArmControlTeleOp extends LinearOpMode {
    DcMotor armMotor = null;

    @Override
    public void runOpMode() {
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");

        // Optionally set motor direction if needed
        armMotor.setDirection(DcMotor.Direction.FORWARD); // Change if necessary
        waitForStart();

        while (opModeIsActive()) {
            // Control the arm with the triggers
            double armPower = gamepad2.right_trigger - gamepad2.left_trigger;

            // Control the arm using buttons for preset positions (optional)
            if (gamepad2.a) {
                armMotor.setPower(0.5); // Move arm up
            } else if (gamepad2.b) {
                armMotor.setPower(-0.5); // Move arm down
            } else {
                armMotor.setPower(0); // Stop the arm
            }

            // Set power to the arm motor based on triggers
            // Uncomment the following line if you want to control it with triggers only
            // armMotor.setPower(armPower);

            // Add a small delay to prevent rapid updates (optional)
            sleep(50);
        }
    }
}
