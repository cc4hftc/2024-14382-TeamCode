package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class ColorTest extends LinearOpMode {
    private ColorSensor colorSensor;
    private DcMotorEx Motor;
    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.colorSensor.get("ColorSensor");
        Motor = hardwareMap.get(DcMotorEx.class, "motor");
        waitForStart();
        while (opModeIsActive()) {
            float red = colorSensor.red();
            float green = colorSensor.green();
            float blue = colorSensor.blue();
            if (red > 20 && red < 50 && green > 50 && green < 80 && blue > 70 && blue < 110) {
                Motor.setPower(-0.1);
            } else if (red > 170 && red < 200 && green > 100 && green < 120 && blue > 50 && blue < 80) {
                Motor.setPower(0.1);
            }
        }
    }
}
