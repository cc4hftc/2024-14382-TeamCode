package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class Auto_Spec extends LinearOpMode {
    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    private IMU imu;
    private DcMotorEx armMotor, otherArmMotor;
    private Servo claw, other_claw, wrist;
    private double targetPosition, otherTargetPosition;
    private double kP = 0.01, kI = 0.00, kD = 0.000, kF = 0.01;
    private double integral = 0, lastError = 0, otherLastError = 0;
    private ElapsedTime timer = new ElapsedTime();
    private boolean pidControlActive = false;
    private int limit = 180;

    @Override
    public void runOpMode() {
        initializeHardware();
        waitForStart();

        while (opModeIsActive()) {
            performAutonomousSequence();
            break;
        }
    }

    private void initializeHardware() {
        // Initialize motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");            // Back
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");              // Left
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");          // Front
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");            // Right

        // Set motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");

        // Initialize arm and servos
        armMotor = hardwareMap.get(DcMotorEx.class, "arm_motor");
        otherArmMotor = hardwareMap.get(DcMotorEx.class, "arm_motor2");
        claw = hardwareMap.get(Servo.class, "clawServo");
        other_claw = hardwareMap.get(Servo.class, "other_clawServo");
        wrist = hardwareMap.get(Servo.class, "wristServo");

        armMotor.setDirection(DcMotorEx.Direction.FORWARD);
        otherArmMotor.setDirection(DcMotorEx.Direction.REVERSE);
        claw.setDirection(Servo.Direction.FORWARD);
        other_claw.setDirection(Servo.Direction.REVERSE);

        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        otherArmMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        otherArmMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        claw.setPosition(0.2075);
        other_claw.setPosition(0.1475);
    }

    private void performAutonomousSequence() {
        //moveAlongYAxis(0.685);
        resetDriveEncoder();
        MoveToTarget(1025);
        stopMotors();
        sleep(250);
        /*moveArmToLimit();
        sleep(150);
        moveWristForward();
        deactivatePID();
        sleep(150);
        openClaw();
        sleep(150);
        moveWristBack();*/
        Turn(540);
        resetDriveEncoder();
        sleep(250);
        MoveToTarget(2150);
        sleep(250);
        Turn(550);
        sleep(500);
    }

    private void moveAlongYAxis(double distance) {
        int targetPosition = (int) (distance * 1440);

        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBackDrive.setTargetPosition(targetPosition);
        rightBackDrive.setTargetPosition(targetPosition);

        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setBackMotorPowers(0.5);

        while (opModeIsActive() && leftBackDrive.isBusy() && rightBackDrive.isBusy()) {
            telemetry.addData("Moving forward", "Distance: %.2f units", distance);
            telemetry.update();
        }

        stopMotors();
    }

    private void moveArmToLimit() {
        armMotor.setTargetPosition(limit);
        otherArmMotor.setTargetPosition(limit);

        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        otherArmMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        armMotor.setPower(0.25);
        otherArmMotor.setPower(0.25);

        while (opModeIsActive() && (armMotor.isBusy() || otherArmMotor.isBusy())) {
            telemetry.addData("Arm1 Target/Pos", "%d/%d", armMotor.getTargetPosition(), armMotor.getCurrentPosition());
            telemetry.addData("Arm2 Target/Pos", "%d/%d", otherArmMotor.getTargetPosition(), otherArmMotor.getCurrentPosition());
            telemetry.update();
        }

        resetPID();
        targetPosition = armMotor.getCurrentPosition();
        otherTargetPosition = otherArmMotor.getCurrentPosition();
        pidControlActive = true;
    }

    private void MoveToTarget(int targetTicks) {
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        resetDriveEncoder();

        while (opModeIsActive()) {
            int currentTicks = leftBackDrive.getCurrentPosition();
            int otherCurrentTicks = rightBackDrive.getCurrentPosition();

            leftBackDrive.setTargetPosition(targetTicks);
            rightBackDrive.setTargetPosition(targetTicks);
            leftFrontDrive.setTargetPosition(0);
            rightFrontDrive.setTargetPosition(0);

            if (currentTicks < targetTicks) {
                setBackMotorPowers(0.3);
            } else if (currentTicks >= targetTicks) {
                stopMotors();
                break;
            }
            telemetry.addData("Current", currentTicks);
            telemetry.addData("Other Current", otherCurrentTicks);
            telemetry.update();
        }
    }


    private void setMotorPowers(double power, boolean clockwise) {
        if (clockwise) {
            // Clockwise: left motors negative, right motors positive
            leftFrontDrive.setPower(-power);
            leftBackDrive.setPower(-power);
            rightFrontDrive.setPower(power);
            rightBackDrive.setPower(power);
        } else {
            // Counterclockwise
            leftFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightFrontDrive.setPower(-power);
            rightBackDrive.setPower(-power);
        }
    }

    private void TurnMotors(double power, boolean clockwise) {
        if (clockwise) {
            // Clockwise: left motors negative, right motors positive
            leftFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightFrontDrive.setPower(power);
            rightBackDrive.setPower(power);
        } else {
            // Counterclockwise
            leftFrontDrive.setPower(-power);
            leftBackDrive.setPower(-power);
            rightFrontDrive.setPower(-power);
            rightBackDrive.setPower(-power);
        }
    }

    private void moveArmDown() {
        limit = 0;
        armMotor.setTargetPosition(limit);
        otherArmMotor.setTargetPosition(limit);

        armMotor.setPower(-0.45);
        otherArmMotor.setPower(-0.45);

        while (opModeIsActive() && (armMotor.isBusy() || otherArmMotor.isBusy())) {
            telemetry.addData("Arm1 Target/Pos", "%d/%d", armMotor.getTargetPosition(), armMotor.getCurrentPosition());
            telemetry.addData("Arm2 Target/Pos", "%d/%d", otherArmMotor.getTargetPosition(), otherArmMotor.getCurrentPosition());
            telemetry.update();
        }

        resetPID();
        targetPosition = armMotor.getCurrentPosition();
        otherTargetPosition = otherArmMotor.getCurrentPosition();
        pidControlActive = false;
    }

    private void moveWristForward() {
        wrist.setPosition(0.475);
        sleep(1500);
    }

    private void moveWristBack() {
        wrist.setPosition(0);
    }

    private void deactivatePID() {
        pidControlActive = false;
        moveArmDown();
        //clipClaw();
        resetPID();

    }

    private void clipClaw() {
        wrist.setPosition(0.55);
    }

    private void openClaw() {
        claw.setPosition(0.095);
        other_claw.setPosition(0.045);
    }

    private void closeClaw() {
        claw.setPosition(0.2075);
        other_claw.setPosition(0.1475);
    }

    private void setBackMotorPowers(double power) {
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
    }

    private void stopMotors() {
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private void resetPID() {
        integral = 0;
        lastError = 0;
        otherLastError = 0;
        timer.reset();
    }

    private double normalizeYaw(double yaw) {
        while (yaw <= -180.0) yaw += 360.0;
        while (yaw > 180.0) yaw -= 360.0;
        return yaw;
    }

    private void Turn(int Ticks) {
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        resetDriveEncoder();

        while (opModeIsActive()) {
            int newCurrentTicks = leftBackDrive.getCurrentPosition();
            int newOtherCurrentTicks = rightBackDrive.getCurrentPosition();

            leftFrontDrive.setTargetPosition(Ticks);
            leftBackDrive.setTargetPosition(Ticks);
            rightFrontDrive.setTargetPosition(Ticks);
            rightBackDrive.setTargetPosition(Ticks);

            if (newCurrentTicks < Ticks) {
                TurnMotors(0.3, true);
            } else if (newCurrentTicks >= Ticks) {
                stopMotors();
                break;
            }
            telemetry.addData("Current", newCurrentTicks);
            telemetry.addData("Other Current", newOtherCurrentTicks);
            telemetry.update();
        }
    }

    private void resetDriveEncoder() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void pidControl() {
        if (!pidControlActive) return;

        double currentPosition = armMotor.getCurrentPosition();
        double otherCurrentPosition = otherArmMotor.getCurrentPosition();

        double error = targetPosition - currentPosition;
        double otherError = otherTargetPosition - otherCurrentPosition;
        double deltaTime = timer.seconds();

        integral += error * deltaTime;
        double derivative = (error - lastError) / deltaTime;

        double power = kP * error + kI * integral + kD * derivative + kF;
        double otherPower = kP * otherError + kI * integral + kD * derivative + kF;

        armMotor.setPower(power);
        otherArmMotor.setPower(otherPower);

        lastError = error;
        otherLastError = otherError;
        timer.reset();
    }
}
