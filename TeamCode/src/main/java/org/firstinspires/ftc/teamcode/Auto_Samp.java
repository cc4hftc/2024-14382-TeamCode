package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous
public class Auto_Samp extends LinearOpMode {
    private PIDController controller;
    public static double p = 0.003, i = 0, d = 0;
    public static double f = 0.004;
    public static int target = 0;
    private final double ticks_in_degree = 288 / 180.0;
    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    private IMU imu;
    private DcMotorEx armMotor, otherArmMotor;
    private DcMotorEx wrist;
    private Servo claw, other_claw;

    @Override
    public void runOpMode() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        initializeHardware();
        waitForStart();

        while (opModeIsActive()) {
            performAutonomousSequence();
            break;
        }
    }


    private void performAutonomousSequence() {
        Strafe(-20);
        sleep(20);
        MoveToTarget(50);
    }

    private void Strafe(int targetTicks) {
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        while (opModeIsActive()) {
            int sCurrentTicks = leftFrontDrive.getCurrentPosition();
            int sOtherCurrentTicks = rightFrontDrive.getCurrentPosition();

            leftBackDrive.setTargetPosition(0);
            rightBackDrive.setTargetPosition(0);
            leftFrontDrive.setTargetPosition(targetTicks);
            rightFrontDrive.setTargetPosition(targetTicks);

            if (sCurrentTicks < targetTicks) {
                setFrontMotorPowers(0.3);
            } else if (sCurrentTicks >= targetTicks) {
                stopMotors();
                break;
            }
        }
    }

    private void MoveToTarget(int Ticks) {
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        resetDriveEncoder();

        while (opModeIsActive()) {
            int CurrentTicks = leftBackDrive.getCurrentPosition();
            int OtherCurrentTicks = rightBackDrive.getCurrentPosition();

            leftBackDrive.setTargetPosition(Ticks);
            rightBackDrive.setTargetPosition(Ticks);
            leftFrontDrive.setTargetPosition(0);
            rightFrontDrive.setTargetPosition(0);

            if (CurrentTicks < Ticks) {
                setBackMotorPowers(0.5);
            } else if (CurrentTicks >= Ticks) {
                stopMotors();
                break;
            }
            telemetry.addData("Current", CurrentTicks);
            telemetry.addData("Other Current", OtherCurrentTicks);
            telemetry.update();
        }
    }

    private void setBackMotorPowers(double power) {
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
    }

    private void setFrontMotorPowers(double power) {
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
    }

    private void stopMotors() {
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
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
        wrist = hardwareMap.get(DcMotorEx.class, "WristMotor");
        claw = hardwareMap.get(Servo.class, "clawServo");
        other_claw = hardwareMap.get(Servo.class, "other_clawServo");

        armMotor.setDirection(DcMotorEx.Direction.FORWARD);
        otherArmMotor.setDirection(DcMotorEx.Direction.REVERSE);
        claw.setDirection(Servo.Direction.FORWARD);
        other_claw.setDirection(Servo.Direction.REVERSE);
        wrist.setDirection(DcMotorEx.Direction.FORWARD);

        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        otherArmMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        otherArmMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        claw.setPosition(0.2075);
        other_claw.setPosition(0.1475);
    }
}
