package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class Just_in_case_we_dont_need_a_auto extends LinearOpMode {
    public void runOpMode() {
        telemetry.addData("I hope your partner is good", "");
        telemetry.update();
        waitForStart();
    }
}