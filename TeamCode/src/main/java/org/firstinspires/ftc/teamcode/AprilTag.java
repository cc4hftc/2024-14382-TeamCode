package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous

public class AprilTag extends LinearOpMode {

    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private HuskyLens huskyLens;
    private IMU imu;
    private int MIN_TARGET_X = 110;
    private int MAX_TARGET_X = 200;

    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");
        imu = hardwareMap.get(IMU.class, "imu");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        if (!huskyLens.knock()) {
            telemetry.addData("HuskyLens", "Communication problem with " + huskyLens.getDeviceName());
        } else {
            huskyLens.initialize();
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
            telemetry.addData("HuskyLens", "Initialized - press start.");
        }
        telemetry.update();
        waitForStart();
        YawPitchRollAngles currentOrientation = imu.getRobotYawPitchRollAngles();
        while (opModeIsActive()) {
            currentOrientation = imu.getRobotYawPitchRollAngles();
            double currentYawNow = normalizeYaw(currentOrientation.getYaw(AngleUnit.DEGREES));
            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);

            for (HuskyLens.Block block : blocks) {
                telemetry.addData("Block", block.toString());
                int x = block.x;

                if (block.id == 2) {
                    // X alignment
                    if (x < MIN_TARGET_X) {
                        // Robot turns left
                        leftFrontDrive.setPower(-0.15);                      // Back
                        leftBackDrive.setPower(-0.15);                      // Left
                        rightFrontDrive.setPower(0.15);                    // Front
                        rightBackDrive.setPower(0.15);                      // Right
                    } else if (x > MAX_TARGET_X) {
                        // Robot turns right
                        leftFrontDrive.setPower(0.2);                     // Back
                        leftBackDrive.setPower(0.2);                       // Left
                        rightFrontDrive.setPower(-0.2);                     // Front
                        rightBackDrive.setPower(-0.2);                     // Right
                    } else {
                        // Stop turning
                        stopMotors();
                    }
                }
            }
        }
    }

    private void stopMotors() {
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private double normalizeYaw(double yaw) {
        while (yaw <= -180.0) {
            yaw += 360.0;
        }
        while (yaw > 180.0) {
            yaw -= 360.0;
        }
        return yaw;
    }
}