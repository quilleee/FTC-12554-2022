package org.firstinspires.ftc.teamcode;
//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name= "ArmTest12554", group="Linear Opmode")
@Disabled

public class ArmTest12554 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor arm = null;

    @Override
    public void runOpMode() throws InterruptedException {

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorEx.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                arm.setPower(0.3);
                telemetry.addData("lift position", arm.getCurrentPosition());
                telemetry.update();
            } else if (gamepad1.y) {
                arm.setPower(-0.3);
                telemetry.addData("lift position", arm.getCurrentPosition());
                telemetry.update();
            } else {
                arm.setPower(0);
                telemetry.addData("lift position", arm.getCurrentPosition());
                telemetry.update();
            }

        }
    }
}
