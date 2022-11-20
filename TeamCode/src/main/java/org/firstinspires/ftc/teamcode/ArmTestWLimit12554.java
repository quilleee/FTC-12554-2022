package org.firstinspires.ftc.teamcode;
//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name= "ArmTestWLimit12554", group="Linear Opmode")
@Disabled

public class ArmTestWLimit12554 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor arm = null;
    DigitalChannel armLimit;

    @Override
    public void runOpMode() throws InterruptedException {

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorEx.Direction.FORWARD);

        armLimit = hardwareMap.get(DigitalChannel.class, "armLimit");
        armLimit.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();

        while (opModeIsActive()) {

            if (armLimit.getState() == false){
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                telemetry.addData("arm position", arm.getCurrentPosition());
                telemetry.update();
            }

            if (gamepad1.a) {
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.setPower(0.5);
                telemetry.addData("arm position", arm.getCurrentPosition());
                telemetry.update();
            } else if (gamepad1.y) {
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.setPower(-0.5);
                telemetry.addData("arm position", arm.getCurrentPosition());
                telemetry.update();
            } else {
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.setPower(0);
                telemetry.addData("lift position", arm.getCurrentPosition());
                telemetry.update();
            }

        }
    }
}
