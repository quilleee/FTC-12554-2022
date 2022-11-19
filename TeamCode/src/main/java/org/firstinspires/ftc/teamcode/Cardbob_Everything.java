package org.firstinspires.ftc.teamcode;



//import com.acmerobotics.dashboard.FtcDashboard;

import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.util.Range;



import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;



@TeleOp(name = "Cardbob_Everything", group = "Concept")

//@Disabled

public class Cardbob_Everything extends LinearOpMode {



    //FtcDashboard dashboard;

    BNO055IMU imu;

    Orientation angles;



    private ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx left1 = null;

    private DcMotorEx left2 = null;

    private DcMotorEx right1 = null;

    private DcMotorEx right2 = null;

    // private DistanceSensor redSideDistance;

    // private DistanceSensor blueSideDistance;

    // private DistanceSensor frontDistance;





    public static PIDFCoefficients DrivetrainPID = new PIDFCoefficients(25, 0, 2.5, 0);

    PIDFCoefficients pidOrig, currentPID;





    public static double P_DRIVE_COEFF = 0.05;

    public static double Target = 70;

    public double greatest_dist = 0;

    //public boolean lifted = false;

    //public boolean secondlift = false;







    static final double COUNTS_PER_MOTOR_REV = 28;

    static final double DRIVE_GEAR_REDUCTION = 12.96;

    static final double WHEEL_DIAMETER_INCHES = 3.779;

    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415926535897932384626)*1.4;



    static final double COUNTS_PER_INCH_STRAFE = COUNTS_PER_INCH /* * a number */;  //((COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415))*0.7874015748; //need to test this number?







    @Override

    public void runOpMode() {

        //dashboard = FtcDashboard.getInstance();

        //Telemetry dashboardTelemetry = dashboard.getTelemetry();



        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);



        left1 = hardwareMap.get(DcMotorEx.class, "left1");

        left2 = hardwareMap.get(DcMotorEx.class, "left2");

        right1 = hardwareMap.get(DcMotorEx.class, "right1");

        right2 = hardwareMap.get(DcMotorEx.class, "right2");



        left1.setDirection(DcMotorEx.Direction.FORWARD);

        left2.setDirection(DcMotorEx.Direction.FORWARD);

        right1.setDirection(DcMotorEx.Direction.REVERSE);

        right2.setDirection(DcMotorEx.Direction.REVERSE);





        resetEncoders();





        left1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        left2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        right1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        right2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);



        pidOrig = left1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);



        left1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DrivetrainPID);

        left2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DrivetrainPID);

        right1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DrivetrainPID);

        right2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DrivetrainPID);



        currentPID = left1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);



        telemetry.addLine("I am ready!");

        telemetry.update();



        waitForStart();



        //    gyroDrive(1, Target, 0, dashboardTelemetry);





        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("axis y", angles.secondAngle);

        telemetry.update();





        gyroDrive(0.5,-40,0);



//resetEncoders();

        while (opModeIsActive()){


            print();

        }



    }





    public void print(){

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("axis y", angles.secondAngle);

        //telemetry.addData("lifted", lifted);

        //telemetry.addData("secondlifted", secondlift);

        telemetry.addData("Position", getCurrentPosition()/COUNTS_PER_INCH);

        telemetry.update();



    }

    public void gyroDrive(double speed, double distance, double angle){



        int newLeftTarget;

        int newRightTarget;

        int moveCounts;

        double max;

        double error;

        double steer;

        double leftSpeed;

        double rightSpeed;



        if (opModeIsActive()) {

            moveCounts = (int) (distance * COUNTS_PER_INCH);

            newLeftTarget = left1.getCurrentPosition() + moveCounts;

            newRightTarget = right1.getCurrentPosition() + moveCounts;



            left1.setTargetPosition(newLeftTarget);

            left2.setTargetPosition(newLeftTarget);

            right1.setTargetPosition(newRightTarget);

            right2.setTargetPosition(newRightTarget);



            left1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            right1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            left2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            right2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);



            speed = Range.clip(Math.abs(speed), 0.0, 1.0);



            while (opModeIsActive() &&

                    left1.isBusy() && right1.isBusy() && left2.isBusy() && right2.isBusy()) {



                print();


                error = getErrorAngle(angle);

                steer = getSteer(error, P_DRIVE_COEFF);



                if (distance < 0)

                    steer *= -1.0;



                leftSpeed = speed - steer;

                rightSpeed = speed + steer;



                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));

                if (max > 1.0) {

                    leftSpeed /= max;

                    rightSpeed /= max;

                }



                left1.setPower(leftSpeed);

                left2.setPower(leftSpeed);



                right1.setPower(rightSpeed);

                right2.setPower(rightSpeed);

/*

                dashboardTelemetry.addData("Left Speed", leftSpeed);

                dashboardTelemetry.addData("Right Speed", rightSpeed);

                print(moveCounts, dashboardTelemetry);*/





            }



            stopMotors();



            left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }



    public double getCurrentPosition() {

        return Math.abs(left1.getCurrentPosition() + left2.getCurrentPosition() + right1.getCurrentPosition() + right2.getCurrentPosition()) / 4;



    }



    public double getErrorAngle(double targetAngle) {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double robotError;



        robotError = targetAngle - angles.firstAngle;

        while (robotError > 180) robotError -= 360;

        while (robotError <= -180) robotError += 360;

        return robotError;

    }



    public double getErrorAngleStrafe(double targetAngle) {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double robotError;



        robotError = targetAngle - angles.firstAngle;

        while (robotError > 180) robotError -= 360;

        while (robotError <= -180) robotError += 360;

        return robotError;

    }





    public double getSteer(double error, double PCoeff) {

        return Range.clip(error * PCoeff, -1, 1);

    }





    public void resetEncoders() {

        left1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        left2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        right1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        right2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

    }

/*

    public void print(double target, Telemetry dashboardTelemetry) {

 

        double dist = getCurrentPosition() / COUNTS_PER_INCH;

 

        if (dist > greatest_dist) {

            greatest_dist = dist;

        }

 

        dashboardTelemetry.addData("Angle", angles.firstAngle);

        dashboardTelemetry.addData("Distance", getCurrentPosition() / COUNTS_PER_INCH);

        dashboardTelemetry.addData("Distance", dist);

        dashboardTelemetry.addData("Peak", greatest_dist);

        dashboardTelemetry.addData("Error", (target - getCurrentPosition()) / COUNTS_PER_INCH);

        dashboardTelemetry.addData("Original PID coef", pidOrig);

        dashboardTelemetry.addData("Current PID coef", currentPID);

        dashboardTelemetry.addData("Tolerance", left2.getTargetPositionTolerance());

        dashboardTelemetry.addData("Peak time", time);

 

 

        dashboardTelemetry.update();

    }*/



    public void stopMotors() {

        left1.setPower(0);

        left2.setPower(0);

        right1.setPower(0);

        right2.setPower(0);

    }



    public void gyroStrafe(double speed, double distance, double angle, Telemetry dashboardTelemetry) {



        resetEncoders();



        double error;

        double steer;



        //int target = (int) (distance * COUNTS_PER_INCH);

        int Left1Target;

        int Right1Target;

        int Left2Target;

        int Right2Target;



        int moveCounts;



        double left1Speed;

        double left2Speed;

        double right1Speed;

        double right2Speed;



        double max1;

        double max2;



        if (opModeIsActive()) {



            moveCounts = (int) (distance * COUNTS_PER_INCH_STRAFE);



            Left1Target = left1.getCurrentPosition() - moveCounts;

            Left2Target = left2.getCurrentPosition() + moveCounts;



            Right1Target = right1.getCurrentPosition() + moveCounts;

            Right2Target = right2.getCurrentPosition() - moveCounts;



            left1.setTargetPosition(Left1Target);

            left2.setTargetPosition(Left2Target);

            right1.setTargetPosition(Right1Target);

            right2.setTargetPosition(Right2Target);



            left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            left2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            right2.setMode(DcMotor.RunMode.RUN_TO_POSITION);



            speed = Range.clip(Math.abs(speed), 0.0, 1.0);

            left1.setPower(speed);

            right1.setPower(speed);

            left2.setPower(speed);

            right2.setPower(speed);



            while (opModeIsActive() && left1.isBusy() && left2.isBusy() && right1.isBusy() && right2.isBusy()) {



                error = getErrorAngleStrafe(angle);

                steer = getSteer(error, P_DRIVE_COEFF);





                if (distance < 0)

                    steer *= -1.0;



                left1Speed = speed + steer;

                left2Speed = speed - steer;



                right1Speed = speed + steer;

                right2Speed = speed - steer;



                max1 = Math.max(Math.abs(left1Speed), Math.abs(right1Speed));

                max2 = Math.max(Math.abs(left2Speed), Math.abs(right2Speed));



                if (max1 > 1.0) {

                    left1Speed /= max1;

                    right1Speed /= max1;



                }

                if (max2 > 1.0) {

                    left2Speed /= max2;

                    right2Speed /= max2;

                }





                left1.setPower(left1Speed);

                left2.setPower(left2Speed);



                right1.setPower(right1Speed);

                right2.setPower(right2Speed);





                //print(distance, error, dashboardTelemetry);



            }

            stopMotors();

            left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            left2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            right2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

 

 

        /*while (opModeIsActive()) {

            // error = target - getCurrentPosition();

            print(target, dashboardTelemetry);

        } */

        }



    }

}

 

 

 