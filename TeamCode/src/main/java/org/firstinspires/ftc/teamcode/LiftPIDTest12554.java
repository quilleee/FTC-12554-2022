package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name= "LiftPIDTest12554", group="Linear Opmode")
@Disabled

public class LiftPIDTest12554 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx lift = null;
    DigitalChannel liftLimit;

    public static PIDFCoefficients LiftPID = new PIDFCoefficients(10,0.49988,0,0); //10, 0.049988, 0, 0 | 23,0,0,0

    //public static PIDFCoefficients LiftPID = new PIDFCoefficients(0,0,0,0); //Ignore the 'f' value, stands for filter, not currently used.
    //PIDFCoefficients currentLiftPID; //Create new PIDCoefficients to set your own PID values and tune.

    //public static  double      Target = 50; //Can set the distance you want to robot to move here.

    static final double     COUNTS_PER_MOTOR_REV    = 28;// HD Hex Motor REV-41-1291
    static final double     DRIVE_GEAR_REDUCTION    = 5.23 * 5.23 * 5.23; // Ratio of gear box 5.23:1
    static final double     WHEEL_DIAMETER_MM   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_MM * 3.1415);
    // there are only three positions (I think)
    // add limit switch
    int Position1 = 0; //home, change later
    int Position2 = 710; //mid, change later
    int Position3 = 2000; //highest, change later

    boolean LiftNeedsToGoFurther = false;

    @Override
    public void runOpMode() throws InterruptedException {
        lift = (DcMotorEx)hardwareMap.get(DcMotorEx.class,"lift");

        liftLimit = hardwareMap.get(DigitalChannel.class,"liftLimit");

        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        // DrivetrainPID is the PID Coefficients set to new values above.
        lift.setDirection(DcMotorEx.Direction.FORWARD);

        liftLimit = hardwareMap.get(DigitalChannel.class, "liftLimit");
        liftLimit.setMode(DigitalChannel.Mode.INPUT);

        lift.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,LiftPID); //10,0.049988,0,0


        waitForStart();

        while (opModeIsActive()){
            // telemetry.addData("lift position", liftLow.getState());
            if (liftLimit.getState() == false){
                lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                telemetry.addData("lift position", lift.getCurrentPosition());
                telemetry.update();
                lift.setTargetPosition(Position1);
                lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            }

            if (gamepad1.a && lift.getCurrentPosition() <= Position3 && liftLimit.getState()) {
                lift.setTargetPosition(Position1);
                lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                //telemetry.addData("PID coefficients",lift.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION));
                telemetry.addData("lift position", lift.getCurrentPosition());
                telemetry.update();
            } else if(gamepad1.b && lift.getCurrentPosition() <= Position3){
                lift.setTargetPosition(Position2);
                lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                //telemetry.addData("PID coefficients",lift.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION));
                telemetry.addData("lift position", lift.getCurrentPosition());
                telemetry.update();
            } else if (gamepad1.y && lift.getCurrentPosition() <= Position3) {
                lift.setTargetPosition(Position3);
                lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                //telemetry.addData("PID coefficients",lift.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION));
                telemetry.addData("lift position", lift.getCurrentPosition());
                telemetry.update();
            }
            //telemetry.addData("PID coefficients",lift.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION));
            telemetry.addData("lift position", lift.getCurrentPosition());
            telemetry.update();
            lift.setPower(0.7);

            if (liftLimit.getState() && lift.getCurrentPosition() >= -100){
                LiftNeedsToGoFurther = true;
            }

            if (LiftNeedsToGoFurther){
                lift.setPower(-0.5);
                LiftNeedsToGoFurther = false;
            }
        }
    }

}