package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous

public class Thing_For_Claw_That_I_Need  extends LinearOpMode {
    private Servo claw = null;
    private Servo other_claw = null;
    private Servo wrist = null;
    @Override
    public void runOpMode() {
        claw          = hardwareMap.get(Servo.class, "clawServo");
        other_claw    = hardwareMap.get(Servo.class, "other_clawServo");
        wrist         = hardwareMap.get(Servo.class, "wristServo");
        claw.setDirection(Servo.Direction.FORWARD);
        other_claw.setDirection(Servo.Direction.REVERSE);
        claw.setPosition(0.2075);
        other_claw.setPosition(0.1475);
        waitForStart();
        while (opModeIsActive()) {
            claw.setPosition(0.095);
            other_claw.setPosition(0.045);
        }
    }
}