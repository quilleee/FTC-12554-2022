package org.firstinspires.ftc.teamcode;
//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name= "LiftTestWLimit12554", group="Linear Opmode")
@Disabled

public class LiftTestWLimit12554 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lift = null;
    DigitalChannel liftLimit;

    @Override
    public void runOpMode() throws InterruptedException {

        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorEx.Direction.FORWARD);

        liftLimit = hardwareMap.get(DigitalChannel.class, "liftLimit");
        liftLimit.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();

        while (opModeIsActive()) {

            if (liftLimit.getState() == false){
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                telemetry.addData("lift position", lift.getCurrentPosition());
                telemetry.update();
            }

            if (gamepad1.a) {
                lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lift.setPower(0.5);
                telemetry.addData("lift position", lift.getCurrentPosition());
                telemetry.update();
            } else if (gamepad1.y) {
                lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lift.setPower(-0.5);
                telemetry.addData("lift position", lift.getCurrentPosition());
                telemetry.update();
            } else {
                lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lift.setPower(0);
                telemetry.addData("lift position", lift.getCurrentPosition());
                telemetry.update();
            }

        }
    }
}
