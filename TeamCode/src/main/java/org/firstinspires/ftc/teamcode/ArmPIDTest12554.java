package org.firstinspires.ftc.teamcode;
//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name= "ArmPIDTest12554", group="Linear Opmode")
@Disabled

public class ArmPIDTest12554 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor arm = null;
    DigitalChannel armLimit;

    int maxArmPosition = -910;
    int minArmPosition = 0;

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
                arm.setTargetPosition(minArmPosition);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (gamepad1.left_bumper) { //down
                arm.setTargetPosition(minArmPosition);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                telemetry.addData("arm position", arm.getCurrentPosition());
                telemetry.update();
            } else if (gamepad1.right_bumper) { //up
                arm.setTargetPosition(maxArmPosition);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                telemetry.addData("arm position", arm.getCurrentPosition());
                telemetry.update();
            }
            arm.setPower(0.6);

        }
    }


}
