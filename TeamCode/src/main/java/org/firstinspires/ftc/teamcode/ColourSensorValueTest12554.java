package org.firstinspires.ftc.teamcode;
//import com.acmerobotics.dashboard.FtcDashboard;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name= "ColourSensorValueTest12554", group="Linear Opmode")
@Disabled

public class ColourSensorValueTest12554 extends LinearOpMode {

    ColorSensor sensorColor;

    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;

    @Override
    public void runOpMode() throws InterruptedException {

        sensorColor = hardwareMap.get(ColorSensor.class, "colourSensor");

        waitForStart();

        while (opModeIsActive()) {


            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

           if (hsvValues[0] < 240 && hsvValues[0] > 160) { //purple
                telemetry.addLine("purple");
            } else if (hsvValues[0] < 65 && hsvValues[0] > 20){ //orange
                telemetry.addLine("orange");
            } else if (hsvValues[0] < 130 && hsvValues[0] > 100){ //green
                telemetry.addLine("green");
            } else{
               telemetry.addLine("other");
            }

            //telemetry.addData("Hue",hsvValues[0]);
            //telemetry.addData("Saturation",hsvValues[1]);
            //telemetry.addData("Value",hsvValues[2]);
            telemetry.update();

        }
    }
}
