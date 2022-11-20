package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name= "LiftArmPIDTest12554", group="Linear Opmode")
@Disabled

public class LiftArmPIDTest12554 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx lift = null;
    DigitalChannel liftLimit;
    DigitalChannel armLimit;
    private DcMotor arm = null;

    int maxArmPosition = -900;
    int minArmPosition = 0;

    public static PIDFCoefficients LiftPID = new PIDFCoefficients(0,0,0,0); //Ignore the 'f' value, stands for filter, not currently used.
    PIDFCoefficients currentLiftPID; //Create new PIDCoefficients to set your own PID values and tune.

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

    @Override
    public void runOpMode() throws InterruptedException {
        lift = hardwareMap.get(DcMotorEx.class,"lift");
        liftLimit = hardwareMap.get(DigitalChannel.class,"liftLimit");

        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // DrivetrainPID is the PID Coefficients set to new values above.
        lift.setDirection(DcMotorEx.Direction.FORWARD);

        liftLimit = hardwareMap.get(DigitalChannel.class, "liftLimit");
        liftLimit.setMode(DigitalChannel.Mode.INPUT);

        PIDFCoefficients LiftPID = new PIDFCoefficients(10.5,0.049988,0,0);

        lift.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,LiftPID); //10,0.049988,0,0

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorEx.Direction.FORWARD);



        waitForStart();

        while (opModeIsActive()){
            // telemetry.addData("lift position", liftLow.getState());
            if (liftLimit.getState() == false){
                lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                //telemetry.addData("lift position", lift.getCurrentPosition());
                telemetry.update();
                lift.setTargetPosition(Position1);
                lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            }

            if (gamepad1.a && lift.getCurrentPosition() <= Position3 && liftLimit.getState()) {
                lift.setTargetPosition(Position1);
                lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                telemetry.addData("PID coefficients",lift.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION));
            } else if(gamepad1.b && lift.getCurrentPosition() <= Position3){
                lift.setTargetPosition(Position2);
                lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                telemetry.addData("PID coefficients",lift.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION));
            } else if (gamepad1.y && lift.getCurrentPosition() <= Position3) {
                lift.setTargetPosition(Position3);
                lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                telemetry.addData("PID coefficients",lift.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION));
            }
            telemetry.addData("PID coefficients",lift.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION));
            telemetry.update();
            lift.setPower(0.7);

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