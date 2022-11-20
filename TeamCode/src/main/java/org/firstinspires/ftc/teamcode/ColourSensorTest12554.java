package org.firstinspires.ftc.teamcode;
//import com.acmerobotics.dashboard.FtcDashboard;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name= "ColourSensorTest12554", group="Linear Opmode")
@Disabled

public class ColourSensorTest12554 extends LinearOpMode {

    ColorSensor sensorColor;

    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;

    @Override
    public void runOpMode() throws InterruptedException {

        sensorColor = hardwareMap.get(ColorSensor.class, "colourSensor");

        waitForStart();

        while (opModeIsActive()) {

           /* if (hsvValues[0] < 0) { //to be changed

            } else if (hsvValues[0] < 0){ //to be changed

            } else if (hsvValues[0] < 0){

            } else{

            } */

            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

            telemetry.addData("Hue",hsvValues[0]);
            telemetry.addData("Saturation",hsvValues[1]);
            telemetry.addData("Value",hsvValues[2]);
            telemetry.update();

            //idk where this is supposed to go
            /* Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues); */
        }
    }
}
