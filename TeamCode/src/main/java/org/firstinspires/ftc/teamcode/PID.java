package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class PID extends LinearOpMode {
    // Motor
    private DcMotorEx wrist = null;

    // Timing
    private ElapsedTime timer = new ElapsedTime();

    // PID constants (you will need to tune these!)
    private double kP = 6;       // Increase if response is too weak, decrease if oscillating
    private double kI = 0.00;      // Increase if steady-state error remains
    private double kD = 1;   //0.25    // Increase if overshoot or oscillation is a problem
    private double kF = 0.003;      // Use if you need a constant offset to hold against gravity

    // PID Variables
    private double targetPosition = 0;  // The encoder position we want to hold
    private double integral = 0;
    private double lastError = 0;

    @Override
    public void runOpMode() {
        // Initialize the wrist motor
        wrist = hardwareMap.get(DcMotorEx.class, "WristMotor");
        wrist.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        wrist.setDirection(DcMotorEx.Direction.FORWARD);
        wrist.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        while (wrist.getCurrentPosition() != 0) {
            idle();
        }
        wrist.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Wait for the Start button
        waitForStart();
        timer.reset();

        // Set the initial target to the current position, so it doesn't jump at the start
        targetPosition = wrist.getCurrentPosition();

        while (opModeIsActive()) {
            // Read the current encoder position
            double currentPosition = wrist.getCurrentPosition();
            // Invert stick if necessary; adjust scale as needed
            double stickInput = -gamepad2.right_stick_y;

            // Manual adjust of the target if the stick is moved
            if (Math.abs(stickInput) > 0.1) {
                // Adjust target up/down by some scaled value each loop
                targetPosition += (stickInput * 0.5);  // <-- TUNE this scaling factor
                resetWristPID(); // Clear integrator and derivative after a manual move
            }

            wristPID(currentPosition);

            if (targetPosition > 120) {
                targetPosition = 119;
            } else if (targetPosition < 0) {
                targetPosition = 1;
            }

            // Send useful data to driver station
            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Error", targetPosition - currentPosition);
            telemetry.addData("kP", kP);
            telemetry.addData("kI", kI);
            telemetry.addData("kD", kD);
            telemetry.addData("kF", kF);
            telemetry.update();
        }
    }

    /**
     * PID control method for the wrist.
     */
    private void wristPID(double currentPosition) {
        double error = targetPosition - currentPosition;

        // Dead zone only applies if the wrist is above the target by 5 or more ticks
        if (error < 0 && Math.abs(error) < 5) {
            wrist.setVelocity(0);  // Stop movement only if it's overshooting
            return;
        }

        if (error == 0 || error == 1) {
            wrist.setPower(0);
            return;
        }

        double dt = timer.seconds();

        // Integral
        integral += (error * dt);

        // Derivative
        double derivative = (error - lastError) / dt;

        // PID output
        double power = kP * error + kI * integral + kD * derivative + kF;

        // Optional power clamp between -1 and 1
        power = Math.max(-1.0, Math.min(1.0, power));

        // Apply power
        wrist.setVelocity(power * 150);

        // Prepare for next iteration
        lastError = error;
        timer.reset();
    }



    /**
     * Resets integral and derivative terms
     * whenever we switch back to a 'new' control mode
     * (e.g., from manual to PID hold).
     */
    private void resetWristPID() {
        integral = 0;
        lastError = 0;
        timer.reset();
    }
}
