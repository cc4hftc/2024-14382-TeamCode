package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous
public class Auto_Samp extends LinearOpMode {
    private PIDController controller;
    public static double p = 0.007, i = 0, d = 0;
    public static double f = 0.004;
    public static int target = 0;
    private final double ticks_in_degree = 288 / 180.0;
    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    private IMU imu;
    private DcMotorEx armMotor, otherArmMotor;
    private DcMotorEx wrist;
    private Servo claw, other_claw;
    private HuskyLens huskyLens;
    private double targetPosition, otherTargetPosition;
    private double kP = 0.01, kI = 0.00, kD = 0.000, kF = 0.01;
    private double integral = 0, lastError = 0, otherLastError = 0;
    private ElapsedTime timer = new ElapsedTime();
    private boolean pidControlActive = false;
    private int limit = 360;
    public static int MIN_TARGET_X = 150;
    public static int MAX_TARGET_X = 160;
    public static int MIN_TARGET_Y = 70;
    public static int MAX_TARGET_Y = 85;
    private int turn = 133;
    private double YSpeed = 0.25;

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
        Strafe(50);
        sleep(80);
        MoveToTarget(830);
        sleep(400);
        ScoreTurn(100);
        sleep(250);
        moveArmToLimit();
        sleep(500);
        moveWristForward();
        sleep(250);
        openClaw();
        sleep(250);
        moveArmToScore();
        sleep(250);
        moveWristBack();
        sleep(100);
        moveArmDown();
        sleep(250);
        Turn(725);
        sleep(250);
        MoveToTarget(750);
        sleep(250);
        openClawWide();
        sleep(50);
        WristSample();
        sleep(250);
        MoveBack(80);
        sleep(250);
        closeClaw();
        sleep(250);
        moveWristBack();
    }

    private void moveArmToLimit() {
        armMotor.setTargetPosition(limit);
        otherArmMotor.setTargetPosition(limit);

        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        otherArmMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        armMotor.setPower(0.3);
        otherArmMotor.setPower(0.3);


        while (opModeIsActive() && (armMotor.isBusy() || otherArmMotor.isBusy())) {
            if (armMotor.getCurrentPosition() < limit && armMotor.getCurrentPosition() > 350) {
                pidControlActive = true;
                break;
            }

            telemetry.addData("Arm1 Target/Pos", "%d/%d", armMotor.getTargetPosition(), armMotor.getCurrentPosition());
            telemetry.addData("Arm2 Target/Pos", "%d/%d", otherArmMotor.getTargetPosition(), otherArmMotor.getCurrentPosition());
            telemetry.update();
        }

        resetPID();
        targetPosition = armMotor.getCurrentPosition();
        otherTargetPosition = otherArmMotor.getCurrentPosition();
        pidControlActive = true;
    }

    private void Strafe(int targetTicks) {
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        resetDriveEncoder();

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
            telemetry.addData("Target", targetTicks);
            telemetry.addData("Pos", sCurrentTicks);
            telemetry.update();
        }
    }

    private void moveArmToScore() {
        limit = 410;
        armMotor.setTargetPosition(limit);
        otherArmMotor.setTargetPosition(limit);

        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        otherArmMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        armMotor.setPower(0.2);
        otherArmMotor.setPower(0.2);


        while (opModeIsActive() && (armMotor.isBusy() || otherArmMotor.isBusy())) {
            if (armMotor.getCurrentPosition() < limit && armMotor.getCurrentPosition() > 350) {
                pidControlActive = true;
                break;
            }

            telemetry.addData("Arm1 Target/Pos", "%d/%d", armMotor.getTargetPosition(), armMotor.getCurrentPosition());
            telemetry.addData("Arm2 Target/Pos", "%d/%d", otherArmMotor.getTargetPosition(), otherArmMotor.getCurrentPosition());
            telemetry.update();
        }

        resetPID();
        targetPosition = armMotor.getCurrentPosition();
        otherTargetPosition = otherArmMotor.getCurrentPosition();
        pidControlActive = true;
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

    private void MoveBack(int Ticks) {
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

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

    private void moveArmDown() {
        limit = 0;
        armMotor.setTargetPosition(limit);
        otherArmMotor.setTargetPosition(limit);

        armMotor.setPower(-0.2);
        otherArmMotor.setPower(-0.2);

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

    private void deactivatePID() {
        pidControlActive = false;
        moveArmDown();
        //clipClaw();
        resetPID();
    }

    private void resetPID() {
        integral = 0;
        lastError = 0;
        otherLastError = 0;
        timer.reset();
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

    private void moveWristForward() {
        target = 125;
        wrist.setTargetPosition(target);
        while (opModeIsActive()) {
            controller.setPID(p, i, d);
            int wristPos = wrist.getCurrentPosition();
            double pid = controller.calculate(wristPos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

            if (wristPos > target) {
                break;
            }

            double power = pid + ff;

            wrist.setPower(power);

            telemetry.addData("Pos ", wristPos);
            telemetry.addData("Target ", target);
            telemetry.update();
        }
    }

    private void moveWristBack() {
        target = 0;
        wrist.setTargetPosition(target);

        while (opModeIsActive()) {
            controller.setPID(p, i, d);
            int wristPos = wrist.getCurrentPosition();
            double pid = controller.calculate(wristPos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

            double power = pid + ff;

            wrist.setPower(power);

            if (wristPos < 0) {
                break;
            }

            if (wristPos > 0 && wristPos < 6) {
                break;
            }

            telemetry.addData("Pos ", wristPos);
            telemetry.addData("Target ", target);
            telemetry.update();
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

    private void ScoreTurn(int Ticks) {
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

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

    private void openClaw() {
        claw.setPosition(0.085);
        other_claw.setPosition(0.035);
    }

    private void openClawWide() {
        claw.setPosition(0.065);
        other_claw.setPosition(0.015);
    }

    private void closeClaw() {
        claw.setPosition(0.2075);
        other_claw.setPosition(0.1475);
    }

    private void WristSample() {
        target = 100;
        wrist.setTargetPosition(target);
        while (opModeIsActive()) {
            controller.setPID(p, i, d);
            int wristPos = wrist.getCurrentPosition();
            double pid = controller.calculate(wristPos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

            double power = pid + ff;

            if (wristPos > target) {
                break;
            }

            wrist.setPower(power);

            telemetry.addData("Pos ", wristPos);
            telemetry.addData("Target ", target);
            telemetry.update();
        }
    }



    private void HuskyInit() {
        if (!huskyLens.knock()) {
            telemetry.addData("HuskyLens", "Communication problem with " + huskyLens.getDeviceName());
        } else {
            huskyLens.initialize();
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
            telemetry.addData("HuskyLens", "Initialized - press start.");
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
        wrist = hardwareMap.get(DcMotorEx.class, "WristMotor");
        claw = hardwareMap.get(Servo.class, "clawServo");
        other_claw = hardwareMap.get(Servo.class, "other_clawServo");

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

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