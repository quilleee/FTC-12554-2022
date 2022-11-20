package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name= "ClawTest12554", group="Linear Opmode")
@Disabled

public class ClawTest12554 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lift = null;
    Servo claw;

    @Override
    public void runOpMode() throws InterruptedException {

        claw = hardwareMap.get(Servo.class,"claw");

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.a){
                claw.setPosition(0.88); //0.9
            } else if(gamepad1.b){
                claw.setPosition(0.65);
            } else if (gamepad1.x){
                claw.setPosition(1);
            }
        }
    }
}
