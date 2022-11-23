package org.firstinspires.ftc.teamcode;
//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/*
*
* NOTE: THIS CODE IS TO BE USED FOR MEASURING THE POSITIONS OF AND TESTING EACH SUBSYSTEM
* FEATURES: POSITION TELEMETRY FOR EVERY NON-DRIVE SUBSYSTEM, NO PID
* NOT DONE YET LOL.
*
* */

@TeleOp(name= "12554 Official Teleop", group="Linear Opmode")

public class Teleop12554 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx lift = null;
    DigitalChannel liftLimit;
    private DcMotor arm = null;
    Servo claw;
    Servo intake;
    DigitalChannel armLimit;

    BNO055IMU imu;

    Orientation angles;

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

    private DcMotor left1 = null;
    private DcMotor right1 = null;
    private DcMotor left2 = null;
    private DcMotor right2 = null;

    double Offset;
    double botHeading;

    //Other setup
    boolean pressed;

    int maxArmPosition = -910;
    int minArmPosition = 0;

    boolean LiftNeedsToGoFurther = false;

    @Override
    public void runOpMode() throws InterruptedException {

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorEx.Direction.FORWARD);

        armLimit = hardwareMap.get(DigitalChannel.class, "armLimit");
        armLimit.setMode(DigitalChannel.Mode.INPUT);

        lift = (DcMotorEx)hardwareMap.get(DcMotorEx.class,"lift");

        liftLimit = hardwareMap.get(DigitalChannel.class,"liftLimit");

        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        // DrivetrainPID is the PID Coefficients set to new values above.
        lift.setDirection(DcMotorEx.Direction.FORWARD);

        liftLimit = hardwareMap.get(DigitalChannel.class, "liftLimit");
        liftLimit.setMode(DigitalChannel.Mode.INPUT);

        lift.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,LiftPID); //10,0.049988,0,0

        claw = hardwareMap.get(Servo.class,"claw");
        intake = hardwareMap.get(Servo.class,"intake");

        left1 = hardwareMap.get(DcMotor.class, "left1");
        right1 = hardwareMap.get(DcMotor.class, "right1");
        left2 = hardwareMap.get(DcMotor.class, "left2");
        right2 = hardwareMap.get(DcMotor.class, "right2");

        left1.setDirection(DcMotor.Direction.REVERSE);
        right2.setDirection(DcMotor.Direction.FORWARD);
        left2.setDirection(DcMotor.Direction.REVERSE);
        right1.setDirection(DcMotor.Direction.FORWARD);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        waitForStart();

        while (opModeIsActive()) {

            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1; //Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;

            // Read inverse IMU heading, as the IMU heading is CW positive.
            double botHeading = getHeading();

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 0.5);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;


            if (gamepad1.y){
                getHeading();
                resetHeading();
            }
            if (gamepad1.left_bumper){
                left1.setPower(frontLeftPower * 0.3);
                left2.setPower(backLeftPower * 0.3);
                right1.setPower(frontRightPower * 0.3);
                right2.setPower(backRightPower * 0.3);
            }  else if (gamepad1.right_bumper){
                left1.setPower(frontLeftPower * 0.65);
                left2.setPower(backLeftPower * 0.65);
                right1.setPower(frontRightPower * 0.65);
                right2.setPower(backRightPower * 0.65);
            }else {
                left1.setPower(frontLeftPower);
                left2.setPower(backLeftPower);
                right1.setPower(frontRightPower);
                right2.setPower(backRightPower);
            }

            if (armLimit.getState() == false){
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                telemetry.addData("arm position", arm.getCurrentPosition());
                telemetry.update();
                arm.setTargetPosition(minArmPosition);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if(gamepad2.y){
                lift.setTargetPosition(Position3);
                lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            } else if (gamepad2.a){
                lift.setTargetPosition(Position1);
                lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            } else if (gamepad2.b){
                lift.setTargetPosition(Position2);
                lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            }/*else if(liftLimit.getState() == false){
                lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                lift.setTargetPosition(Position1);
                lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                telemetry.addData("ZERO", true);

            }*/

            telemetry.addData("button", liftLimit.getState());
            telemetry.update();
            lift.setPower(1);





            if (gamepad2.left_bumper) { //down
                arm.setTargetPosition(minArmPosition);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                telemetry.addData("arm position", arm.getCurrentPosition());
                telemetry.update();
            } else if (gamepad2.right_bumper) { //up
                arm.setTargetPosition(maxArmPosition);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                telemetry.addData("arm position", arm.getCurrentPosition());
                telemetry.update();
            }
            arm.setPower(0.6);

            if(gamepad1.left_bumper){
                claw.setPosition(0.95); //0.88
            } else if(gamepad1.right_bumper){
                claw.setPosition(0.65);
            }


            if(gamepad1.b){
                intake.setPosition(0.3);
                telemetry.addData("intake position", intake.getPosition());
                telemetry.update();
            }

        }
    }
    public void resetHeading(){

        Offset = imu.getAngularOrientation().firstAngle;

    }
    public double getHeading(){

        botHeading = -imu.getAngularOrientation().firstAngle + Offset;
        return botHeading;
    }

}
