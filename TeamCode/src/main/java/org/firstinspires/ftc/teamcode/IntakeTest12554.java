package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name= "IntakeTest12554", group="Linear Opmode")
@Disabled

public class IntakeTest12554 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lift = null;
    Servo intake;

    @Override
    public void runOpMode() throws InterruptedException {

        intake = hardwareMap.get(Servo.class,"intake");

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.a){
                intake.setPosition(1);
            } else if(gamepad1.b){
                intake.setPosition(0.3);
            }
        }
    }
}
