
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//@Config
@Autonomous(name="Autonomous_PIDStrafe", group="Iterative Opmode")
//@Disabled
public class Autonomous_PIDStrafe extends LinearOpMode {

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

    @Override
    public void runOpMode(){

        //dashboard = FtcDashboard.getInstance();
        //Telemetry dashboardTelemetry = dashboard.getTelemetry();

        left1=hardwareMap.get(DcMotorEx.class,"left1"); //motor 0
        right1=hardwareMap.get(DcMotorEx.class,"right1"); //motor 0
        left2=hardwareMap.get(DcMotorEx.class,"left2"); //motor 0
        right2=hardwareMap.get(DcMotorEx.class,"right2");

        sensorColor = hardwareMap.get(ColorSensor.class, "colourSensor");


        left1.setDirection(DcMotor.Direction.FORWARD);
        right1.setDirection(DcMotor.Direction.REVERSE);
        left2.setDirection(DcMotor.Direction.FORWARD);
        right2.setDirection(DcMotor.Direction.REVERSE);

        left1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        left2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        left1.setTargetPositionTolerance(15);
        left2.setTargetPositionTolerance(15);
        right1.setTargetPositionTolerance(15);
        right2.setTargetPositionTolerance(15);


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

        telemetry.addData(">","Robot Ready.");
        telemetry.update();
        waitForStart();

        gyroDrive(0.6, 550);
        detectColour();
        goToLocation(location);


        while(opModeIsActive()){

            print((int)(0.6 * COUNTS_PER_MM_STRAFE));



        }
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

            ramp += 0.05;

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
        double kP = 0.1;
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

        stopMotors();
        telemetry.addLine("Reached position");

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

        //telemetry.addData("Distance", dist);
        //telemetry.addData("Error", (target- getCurrentPosition())/COUNTS_PER_MM_STRAFE);
        //telemetry.addData("ErrorAngle", angles.firstAngle);

        //telemetry.update();
    }

    public int detectColour(){

        telemetry.addLine("Detecting colour...");
        // use timer
        // while
        runtime.reset();
        while (runtime.time() < 2 ){
            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

            if (hsvValues[2] > 150){
                if (hsvValues[0] < 40 || hsvValues[0] > 320) { //pink
                    location = 1;
                    telemetry.addLine("pink");
                } else if (hsvValues[0] < 180 && hsvValues[0] > 140){ //green
                    location = 2;
                    telemetry.addLine("green");
                } else if (hsvValues[0] < 130 && hsvValues[0] > 70){ //yellow
                    location = 3;
                    telemetry.addLine("yellow");
                } else{
                    telemetry.addLine("other");
                }
            } else {
                telemetry.addLine("No colours detected");
            }



            telemetry.update();

        }

        return location;

    }

    public void goToLocation(int location){

        gyroDrive(0.6, 660);

        if (location == 1){
            gyroStrafe(0.6,true,600);
            telemetry.addLine("Location 1: move left");
        } else if (location == 2){
            telemetry.addLine("Location 2: hold");
        } else if (location == 3){
            gyroStrafe(0.6, false,600);
            telemetry.addLine("Location 3: move right");
        }

        telemetry.update();
    }




}