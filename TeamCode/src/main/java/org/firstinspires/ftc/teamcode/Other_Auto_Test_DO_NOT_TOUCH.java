package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Autonomous
public class Other_Auto_Test_DO_NOT_TOUCH extends LinearOpMode {

    private final int READ_PERIOD = 1;

    private HuskyLens huskyLens;

    // Declare motor variables
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

    @Override
    public void runOpMode() {
        // Initialize HuskyLens and motors
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Set initial motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        // Set algorithm for AprilTag recognition
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();
            
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            
            // Get detected blocks
            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());
                int x = (blocks[i].x);
                int y = (blocks[i].y);
                telemetry.addData("x", x);
                telemetry.addData("y", y);

                // Check if the detected block has ID
                if (blocks[i].id == 2) {
                    // Move motors accordingly
                    // Set motor directions: 2 forward, 2 backward
                    /*leftFrontDrive.setPower((x/200)*-1); //back
                    leftBackDrive.setPower((y/200)*-1); //Right
                    rightFrontDrive.setPower((x/200)*-1); //front
                    rightBackDrive.setPower((y/200)*-1); //Left*/
                    
                    double motorfront = ((x/200)*-1);
                    double motorback = ((y/200)*-1);
                    double motorleft = ((x/200)*-1);
                    double motorright = ((y/200)*-1);
                    telemetry.addData("motorfront", motorfront);
                    telemetry.addData("motorback", motorback);
                    telemetry.addData("motorleft", motorleft);
                    telemetry.addData("motorright", motorright);
                }
            }
            telemetry.update();
        }
    }
}
