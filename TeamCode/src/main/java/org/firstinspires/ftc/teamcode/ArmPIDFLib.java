package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class ArmPIDFLib extends LinearOpMode {
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    public static int limit = 300;

    public static int ArmST = 2;

    public int TriggerPosition = 0;

    private final double ticks_in_degree = (double) 840 /360;

    private DcMotorEx armMotor;
    private DcMotorEx otherArmMotor;

    @Override
    public void runOpMode() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armMotor = hardwareMap.get(DcMotorEx.class, "arm_motor");
        otherArmMotor = hardwareMap.get(DcMotorEx.class, "arm_motor2");

        armMotor.setDirection(DcMotorEx.Direction.FORWARD);
        otherArmMotor.setDirection(DcMotorEx.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            target = TriggerPosition;

            controller.setPID(p, i, d);
            int armPos = armMotor.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

            double power = pid + ff;

            TriggersMove();

            armMotor.setPower(power);
            otherArmMotor.setPower(power);

            telemetry.addData("Pos ", armPos);
            telemetry.addData("Target ", target);
            telemetry.update();
        }
    }

    private void TriggersMove() {
        double Triggers = gamepad2.left_trigger - gamepad2.right_trigger;
        if (TriggerPosition >= limit) {
            TriggerPosition-=1;
        }

        if (Triggers > 0.1) {
            TriggerPosition += 1;
            sleep(ArmST);
        } else if (Triggers < -0.1) {
            TriggerPosition -= 1;
            sleep(ArmST);
        } else if (Triggers == 0) {
            // Do nothing
        }
    }
}
