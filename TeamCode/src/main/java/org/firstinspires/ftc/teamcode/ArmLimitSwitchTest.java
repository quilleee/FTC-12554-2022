package org.firstinspires.ftc.teamcode;
//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name= "ArmLimitSwitchTest", group="Linear Opmode")
@Disabled

public class ArmLimitSwitchTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor arm = null;
    DigitalChannel armLimit;

    int maxArmPosition = -730;
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
                telemetry.addLine("false");
                telemetry.update();

            } else if(armLimit.getState() == true){
                telemetry.addLine("true");
                telemetry.update();
            }

        }
    }

}
