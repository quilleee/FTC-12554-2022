
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//@Config
@Autonomous(name="Autonomous_PIDMovement", group="Iterative Opmode")
//@Disabled
public class Autonomous_PIDMovement extends LinearOpMode {

    /**
     * ---------------------------- FTC DASHBOARD ------------------------------------
     * https://acmerobotics.github.io/ftc-dashboard/
     * To open the dashboard connect your laptop to the robot's wifi and access this address using a browser:
     * http://192.168.43.1:8080/dash
     *
     */

    //FtcDashboard dashboard;

    BNO055IMU imu;
    Orientation angles;

    // Declare OpMode members.
    private ElapsedTime runtime=new ElapsedTime();

    // Gyro for strafe
    // p controller with gyro
    // depends on whether left or right (boolean)
    // distance var
    // Find strafe constant
    // set target pos based off boolean
    // error is target - imu
    // order is xzy
    // acceleration speed = speed + x
    // gyro p = y * error * kp
    // boolean affects: front neg, back pos
    // speed capped at 0.5 for autonomous

    public static PIDFCoefficients DrivetrainPID=new PIDFCoefficients(12,0,1.2,0);
    // Tuning: >20 is too much
    // p-15, error-0.78 (799)
    // p-12, error-
    // 0.5, 3
    // d-1.2, 3
    // d-1.8, 4

    // 1 tile: 596.9 millimeters

    // Calculating
    static final double COUNTS_PER_MOTOR_REV = 28; // HD Hex Motor REV-41-1291
    static final double DRIVE_GEAR_REDUCTION = 27.3529; // 5.23^2 (Ratio of gear box 5.23:1)
    static final double WHEEL_DIAMETER_MM = 96; //goBILDA 96mm Mecanum Wheel
    static final double COUNTS_PER_MM_STRAFE=((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER_MM*3.1415));
    static final double COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_MM * 3.1415);

    private DcMotorEx left1=null;
    private DcMotorEx right1=null;
    private DcMotorEx left2=null;
    private DcMotorEx right2=null;

    ColorSensor sensorColor;

    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;

    double error;
    int location;
    // print error (target - current position)/ counts per inch

    // Other parts
    private DcMotorEx lift = null;
    DigitalChannel liftLimit;
    private DcMotor arm = null;
    DigitalChannel armLimit;
    Servo claw;

    // Lift PID
    int Position1 = 0;
    int Position2 = 710;
    int Position3 = 2000;

    public static PIDFCoefficients LiftPID = new PIDFCoefficients(10,0.49988,0,0); //10, 0.049988, 0, 0 | 23,0,0,0

    // Arm
    int maxArmPosition = -910;
    int minArmPosition = 0;


    @Override
    public void runOpMode(){

        //dashboard = FtcDashboard.getInstance();
        //Telemetry dashboardTelemetry = dashboard.getTelemetry();

        left1=hardwareMap.get(DcMotorEx.class,"left1"); //motor 0
        right1=hardwareMap.get(DcMotorEx.class,"right1"); //motor 0
        left2=hardwareMap.get(DcMotorEx.class,"left2"); //motor 0
        right2=hardwareMap.get(DcMotorEx.class,"right2");

        sensorColor = hardwareMap.get(ColorSensor.class, "colourSensor");

        // Arm
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorEx.Direction.FORWARD);

        armLimit = hardwareMap.get(DigitalChannel.class, "armLimit");
        armLimit.setMode(DigitalChannel.Mode.INPUT);

        // Lift
        lift = (DcMotorEx)hardwareMap.get(DcMotorEx.class,"lift");

        liftLimit = hardwareMap.get(DigitalChannel.class,"liftLimit");

        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        // DrivetrainPID is the PID Coefficients set to new values above.
        lift.setDirection(DcMotorEx.Direction.FORWARD);

        liftLimit = hardwareMap.get(DigitalChannel.class, "liftLimit");
        liftLimit.setMode(DigitalChannel.Mode.INPUT);

        lift.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,LiftPID); //10,0.049988,0,0


        // Other
        claw = hardwareMap.get(Servo.class,"claw");

        // Drivetrain
        left1.setDirection(DcMotor.Direction.FORWARD);
        right1.setDirection(DcMotor.Direction.REVERSE);
        left2.setDirection(DcMotor.Direction.FORWARD);
        right2.setDirection(DcMotor.Direction.REVERSE);

        left1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        left2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //left1.setTargetPositionTolerance(20);
        //left2.setTargetPositionTolerance(20);
        //right1.setTargetPositionTolerance(20);
        //,.right2.setTargetPositionTolerance(20);

        left1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DrivetrainPID);
        left2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DrivetrainPID);
        right1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DrivetrainPID);
        right2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DrivetrainPID);

        BNO055IMU.Parameters parameters=new BNO055IMU.Parameters();
        parameters.angleUnit=BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit=BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile="BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled=true;
        parameters.loggingTag="IMU";
        parameters.accelerationIntegrationAlgorithm=new JustLoggingAccelerationIntegrator();

        imu=hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);

        //telemetry.addData(">","Robot Ready.");    //
        //telemetry.update();
        waitForStart();

        // Lower intake (no)
        // Lift arm and lift??
        // Raise intake (no)
        // Move to cone
        // Move to end
        // Strafe
        // Raise lift
        // open claw
        //detectColour();
        //raiseLift(Position1);

        gyroStrafe(0.6,false,600);
        strafeCorrection(false);


        while(opModeIsActive()){

            print((int)(0.6 * COUNTS_PER_MM_STRAFE));



        }
    }

    public void raiseLift(int liftPosition){

        lift.setTargetPosition(liftPosition);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void raiseArm(int armPosition){

        arm.setTargetPosition(armPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("arm position", arm.getCurrentPosition());
    }


    public void gyroDrive(double maxSpeed, double distance){

        resetEncoders();

        int target =(int)(distance * COUNTS_PER_MM);

        left1.setTargetPosition(target);
        left2.setTargetPosition(target);
        right1.setTargetPosition(target);
        right2.setTargetPosition(target);

        left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double p;
        double kP = 0.1;
        double y = 0.5;
        double ramp = 0;

        while (opModeIsActive() && left1.isBusy() && left2.isBusy() && right1.isBusy()  && right2.isBusy()){
            // add telemetry
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZXY,AngleUnit.DEGREES);

            ramp += 0.2;


            if (ramp > 1 ) {
                ramp = 1;
            }

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZXY,AngleUnit.DEGREES);
            double AngleError = 0 - angles.firstAngle;
            p = AngleError * kP * y;

            left1.setPower((maxSpeed - p) * ramp);
            left2.setPower((maxSpeed - p) * ramp);
            right1.setPower((maxSpeed + p) * ramp);
            right2.setPower((maxSpeed + p) * ramp);

            error = target - getCurrentPosition();

            print(target);


        }
        stopMotors();

    }


    public void gyroStrafe(double maxSpeed, boolean isleft, double distance){

        resetEncoders();

        double StrafeConstant = 1;

        int target =(int)(distance * COUNTS_PER_MM_STRAFE * StrafeConstant);

        if (isleft) {
            left1.setTargetPosition(-target);
            left2.setTargetPosition(target);
            right1.setTargetPosition(target);
            right2.setTargetPosition(-target);
        } else {
            left1.setTargetPosition(target);
            left2.setTargetPosition(-target);
            right1.setTargetPosition(-target);
            right2.setTargetPosition(target);
        }

        left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double p;
        double kP = 0.2;
        // 0.05: 3.9
        // 0.1: -9.7 / 4.6
        // 0.2: -9.2/ 4.8
        // 0.3: / 4.8
        // 0.4: -8.9/ 5.5
        // 0.5: -8.8/ 6.2
        // 0.6: -9.5
        double y = 0.5;
        // 5.18
        // 1- 4.57

        double ramp = 0;

      //https://fll-pigeons.github.io/gamechangers/gyro_pid.html (uhhh this helps)

        while (opModeIsActive() && left1.isBusy() && left2.isBusy() && right1.isBusy() && right2.isBusy()){
            // add telemetry

            ramp += 0.05;

            if (ramp > 1 ) {
                ramp = 1;
            }
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZXY,AngleUnit.DEGREES);
            double AngleError = 0 - angles.firstAngle;
            p = AngleError * kP * y;

            if ( maxSpeed > 0.6) {
                maxSpeed = 0.6;
            }

            if (isleft) {
                left1.setPower((maxSpeed + p)*ramp);
                left2.setPower((maxSpeed - p)*ramp);
                right1.setPower((maxSpeed + p)*ramp);
                right2.setPower((maxSpeed - p)*ramp);
            } else {
                left1.setPower((maxSpeed - p)*ramp);
                left2.setPower((maxSpeed + p)*ramp);
                right1.setPower((maxSpeed - p)*ramp);
                right2.setPower((maxSpeed + p)*ramp);
            }

            error = target - getCurrentPosition();

            print(target);



        }
        telemetry.update();

        stopMotors();

    }

    public void strafeCorrection(boolean isLeft){

        resetEncoders();
        resetRuntime();

        double p;
        double P = 0.2;
        double turnSpeed = 0.6;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZXY,AngleUnit.DEGREES);
        double AngleError = 0 - angles.firstAngle;
        p = AngleError * P;

        while (angles.firstAngle > 2) {

            while (runtime.time() < 3) {

                if (isLeft){

                    left1.setPower(turnSpeed * p);
                    left2.setPower(turnSpeed * p);
                    right1.setPower(-turnSpeed * p);
                    right2.setPower(-turnSpeed * p);

                } else {

                    left1.setPower(-turnSpeed * p);
                    left2.setPower(-turnSpeed * p);
                    right1.setPower(turnSpeed * p);
                    right2.setPower(turnSpeed * p);

                }

            }

        }

        }

    public void resetEncoders(){
        left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double getCurrentPosition(){
        return (left1.getCurrentPosition() + left2.getCurrentPosition() + right1.getCurrentPosition() + right2.getCurrentPosition())/4; // Math.abs returns the absolute value of an argument
    }

    public void stopMotors(){

        left1.setPower(0);
        right1.setPower(0);
        left2.setPower(0);
        right2.setPower(0);

    }


    public void print(double target){

        double dist = getCurrentPosition()/COUNTS_PER_MM_STRAFE;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZXY,AngleUnit.DEGREES);

        telemetry.addData("Distance", dist);
        telemetry.addData("Error", (target- getCurrentPosition())/COUNTS_PER_MM_STRAFE);
        telemetry.addData("ErrorAngle", angles.firstAngle);

        telemetry.update();
    }

    public int detectColour(){
        // use timer
        // while
        resetRuntime();
        while (runtime.time() < 1 ){
            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

            if (hsvValues[0] < 240 && hsvValues[0] > 160) { //purple
                location = 1;
                telemetry.addLine("purple");
            } else if (hsvValues[0] < 65 && hsvValues[0] > 20){ //orange
                location = 2;
                telemetry.addLine("orange");
            } else if (hsvValues[0] < 120 && hsvValues[0] > 100){ //green
                location = 3;
                telemetry.addLine("green");
            } else{
                telemetry.addLine("other");
            }

            telemetry.update();

        }

        return location;

    }




}