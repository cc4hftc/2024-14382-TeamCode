package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp

public class RunToPosition extends LinearOpMode {
    private DcMotorEx wrist = null;
    
    @Override
    public void runOpMode() {
        wrist = hardwareMap.get(DcMotorEx.class, "WristMotor");
        wrist.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        wrist.setDirection(DcMotorEx.Direction.FORWARD);
        wrist.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        while (wrist.getCurrentPosition() != 0) {
            idle();
        }
        
        wrist.setTargetPosition(30);
        wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        waitForStart();
        
        while (opModeIsActive()) {
            telemetry.addData("Current", wrist.getCurrentPosition());
            telemetry.update();
            wrist.setVelocity(50);
        }
    }
}
