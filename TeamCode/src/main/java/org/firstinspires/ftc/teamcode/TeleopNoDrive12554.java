package org.firstinspires.ftc.teamcode;
//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
*
* NOTE: THIS CODE IS TO BE USED FOR MEASURING THE POSITIONS OF AND TESTING EACH SUBSYSTEM
* FEATURES: SUBSYSTEMS MOVE, ALL MOTORS HAVE PID
*
* */

@TeleOp(name= "TeleopNoDrive12554", group="Linear Opmode")
@Disabled

public class TeleopNoDrive12554 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lift = null;
    DigitalChannel liftLimit;
    private DcMotor arm = null;
    Servo claw;
    Servo intake;
    DigitalChannel armLimit;

    @Override
    public void runOpMode() throws InterruptedException {

        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorEx.Direction.FORWARD);

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorEx.Direction.FORWARD);

        armLimit = hardwareMap.get(DigitalChannel.class, "liftLimit");
        armLimit.setMode(DigitalChannel.Mode.INPUT);

        liftLimit = hardwareMap.get(DigitalChannel.class, "liftLimit");
        liftLimit.setMode(DigitalChannel.Mode.INPUT);

        claw = hardwareMap.get(Servo.class,"claw");
        intake = hardwareMap.get(Servo.class,"intake");

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

            if (armLimit.getState() == false){
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                telemetry.addData("arm position", arm.getCurrentPosition());
                telemetry.update();
            }

            if (gamepad1.left_bumper) {
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.setPower(0.5);
                telemetry.addData("arm position", arm.getCurrentPosition());
                telemetry.update();
            } else if (gamepad1.right_bumper) {
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

            if(gamepad2.left_bumper){
                claw.setPosition(1);
                telemetry.addData("claw position", claw.getPosition());
                telemetry.update();
            } else if(gamepad1.right_bumper){
                claw.setPosition(0.85);
                telemetry.addData("claw position", claw.getPosition());
                telemetry.update();
            }

            if(gamepad2.left_stick_button){
                intake.setPosition(1);
                telemetry.addData("intake position", intake.getPosition());
                telemetry.update();
            } else if(gamepad2.right_stick_button){
                intake.setPosition(0.3);
                telemetry.addData("intake position", intake.getPosition());
                telemetry.update();
            }

        }
    }
}
