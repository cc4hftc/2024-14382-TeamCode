package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp

public class Velocity extends LinearOpMode {
    private DcMotorEx wrist = null;
    
     @Override
    public void runOpMode() {
        wrist = hardwareMap.get(DcMotorEx.class, "WristMotor");
        wrist.setDirection(DcMotorEx.Direction.FORWARD);
        wrist.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        
        
        waitForStart();
        
        while (opModeIsActive()) {
            telemetry.addData("Current", wrist.getCurrentPosition());
            telemetry.update();
            RunToTarget(500);
        }
    }
    
    private void RunToTarget(int targetPosition) {
        wrist.setTargetPosition(targetPosition);
        
        wrist.setVelocity(targetPosition);
    }
}
