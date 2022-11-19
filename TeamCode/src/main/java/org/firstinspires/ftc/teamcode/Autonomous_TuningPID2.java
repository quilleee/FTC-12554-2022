/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//@Config
@Autonomous(name="Autonomous_TuningPID2", group="Iterative Opmode")
//@Disabled
public class Autonomous_TuningPID2 extends LinearOpMode {

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

    public static PIDFCoefficients DrivetrainPID=new PIDFCoefficients(5,0,0,0);
    public static int target = 50;

    // Calculating
    static final double COUNTS_PER_MOTOR_REV = 28; // HD Hex Motor REV-41-1291
    static final double DRIVE_GEAR_REDUCTION = 27.3529; // 5.23^2 (Ratio of gear box 5.23:1)
    static final double WHEEL_DIAMETER_MM = 96; //goBILDA 96mm Mecanum Wheel
    static final double COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                        (WHEEL_DIAMETER_MM * 3.1415);

    private DcMotorEx left1=null;
    private DcMotorEx right1=null;
    private DcMotorEx left2=null;
    private DcMotorEx right2=null;

    double error;
    public double greatest_dist = 0;

    // print error (target - current position)/ counts per inch

    @Override
    public void runOpMode(){

        //dashboard = FtcDashboard.getInstance();
        //Telemetry dashboardTelemetry = dashboard.getTelemetry();

        left1=hardwareMap.get(DcMotorEx.class,"left1"); //motor 0
        right1=hardwareMap.get(DcMotorEx.class,"right1"); //motor 0
        left2=hardwareMap.get(DcMotorEx.class,"left2"); //motor 0
        right2=hardwareMap.get(DcMotorEx.class,"right2");

        left1.setDirection(DcMotor.Direction.FORWARD);
        right1.setDirection(DcMotor.Direction.REVERSE);
        left2.setDirection(DcMotor.Direction.FORWARD);
        right2.setDirection(DcMotor.Direction.REVERSE);

        resetEncoders();

        telemetry.addData(">","Robot Ready.");    //
        telemetry.update();

        left1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        left2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        left1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DrivetrainPID);
        left2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DrivetrainPID);
        right1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DrivetrainPID);
        right2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DrivetrainPID);

        int relativeLayoutId=hardwareMap.appContext.getResources().getIdentifier("RelativeLayout","id",hardwareMap.appContext.getPackageName());
        final View relativeLayout=((Activity)hardwareMap.appContext).findViewById(relativeLayoutId);

        BNO055IMU.Parameters parameters=new BNO055IMU.Parameters();
        parameters.angleUnit=BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit=BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile="BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled=true;
        parameters.loggingTag="IMU";
        parameters.accelerationIntegrationAlgorithm=new JustLoggingAccelerationIntegrator();

        imu=hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);

        waitForStart();

        gyroDrive(0.8, 500);

        while(opModeIsActive()){

            print((int)(0.8 * COUNTS_PER_MM), telemetry);

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

        //https://fll-pigeons.github.io/gamechangers/gyro_pid.html (uhhh this helps)

        while (opModeIsActive() && left1.isBusy() && left2.isBusy() && right1.isBusy()  && right2.isBusy()){
            // add telemetry

            double leftPower = maxSpeed; // change power based off angle error
            double rightPower = maxSpeed;

            left1.setPower(leftPower);
            left2.setPower(leftPower);
            right1.setPower(rightPower);
            right2.setPower(rightPower);

            error = target - getCurrentPosition();

            print(target, telemetry);


        }
        stopMotors();

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
        left2.setPower(0);
        right1.setPower(0);
        right2.setPower(0);

    }


    public void print(double target,Telemetry telemetry){

        double dist = getCurrentPosition()/COUNTS_PER_MM;

        telemetry.addData("Distance", dist);
        telemetry.addData("Error", (target- getCurrentPosition())/COUNTS_PER_MM);

        telemetry.update();
    }




}