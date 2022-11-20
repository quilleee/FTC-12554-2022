package org.firstinspires.ftc.teamcode;
//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name= "LiftArmTest12554", group="Linear Opmode")
@Disabled

public class LiftArmTest12554 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lift = null;
    DigitalChannel liftLimit;
    DigitalChannel armLimit;
    private DcMotor arm = null;

    int maxArmPosition = -900;
    int minArmPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorEx.Direction.FORWARD);

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorEx.Direction.FORWARD);

        liftLimit = hardwareMap.get(DigitalChannel.class, "liftLimit");
        liftLimit.setMode(DigitalChannel.Mode.INPUT);

        armLimit = hardwareMap.get(DigitalChannel.class, "armLimit");
        armLimit.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();

        while (opModeIsActive()) {

            if (liftLimit.getState() == false){
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                telemetry.addData("lift position", lift.getCurrentPosition());
                telemetry.update();

            }

            if (gamepad1.y) {
                lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lift.setPower(0.5);
                telemetry.addData("lift position", lift.getCurrentPosition());
                telemetry.update();
            } else if (gamepad1.a) {
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
