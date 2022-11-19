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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.Locale;


@Autonomous(name="Autonomous_PID", group="Iterative Opmode")
@Disabled
public class Autonomous_PID extends LinearOpMode {

    BNO055IMU imu;

    Orientation angles;

    /* Declare OpMode members. */
    private ElapsedTime runtime=new ElapsedTime();
    public static PIDFCoefficients DrivetrainPID=new PIDFCoefficients(25,0.05,1.25,0);
    PIDFCoefficients pidOrig, currentPID;

    static final double COUNTS_PER_MOTOR_REV=746.6;    //
    static final double WHEEL_DIAMETER_INCHES=4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH=((COUNTS_PER_MOTOR_REV)/(WHEEL_DIAMETER_INCHES*3.1415))/1.65;
    static final double COUNTS_PER_INCH_STRAFE=((COUNTS_PER_MOTOR_REV)/(WHEEL_DIAMETER_INCHES*3.1415))*0.7874015748;

    private DcMotorEx leftDrive1=null;
    private DcMotorEx rightDrive1=null;
    private DcMotorEx leftDrive2=null;
    private DcMotorEx rightDrive2=null;


    static final double HEADING_THRESHOLD=1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF=0.025;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF=0.04;     // Larger is more responsive, but also less stable

    @Override
    public void runOpMode(){

        //Rightbox = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

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

        leftDrive1=hardwareMap.get(DcMotorEx.class,"left_drive1"); //motor 0
        rightDrive1=hardwareMap.get(DcMotorEx.class,"right_drive1"); //motor 0
        leftDrive2=hardwareMap.get(DcMotorEx.class,"left_drive2"); //motor 0
        rightDrive2=hardwareMap.get(DcMotorEx.class,"right_drive2"); //motor 0

        leftDrive1.setDirection(DcMotor.Direction.FORWARD);
        rightDrive1.setDirection(DcMotor.Direction.REVERSE);
        leftDrive2.setDirection(DcMotor.Direction.FORWARD);
        rightDrive2.setDirection(DcMotor.Direction.REVERSE);


        // Ensure the robot it stationary, then reset the encoders
        resetEncoders();

        telemetry.addData(">","Robot Ready.");    //
        telemetry.update();

        leftDrive1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftDrive2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightDrive1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightDrive2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftDrive1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,DrivetrainPID);
        leftDrive2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,DrivetrainPID);
        rightDrive1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,DrivetrainPID);
        rightDrive2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,DrivetrainPID);

        currentPID = leftDrive1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();


        gyroDrive(0.2,-45,0);
        gyroHold(0,0,1); //pause for 1 second

        printEncoders();
        telemetry.update();

    }

    public void resetEncoders(){
        leftDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void gyroDrive(double speed,double distance,double angle){

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if(opModeIsActive()){

            // Determine new target position, and pass to motor controller
            moveCounts=(int)(distance*COUNTS_PER_INCH);
            newLeftTarget=leftDrive1.getCurrentPosition()+moveCounts;
            newRightTarget=rightDrive1.getCurrentPosition()+moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            leftDrive1.setTargetPosition(newLeftTarget);
            rightDrive1.setTargetPosition(newRightTarget);
            leftDrive2.setTargetPosition(newLeftTarget);
            rightDrive2.setTargetPosition(newRightTarget);

            leftDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed=Range.clip(Math.abs(speed),0.0,1.0);
            leftDrive1.setPower(speed);
            rightDrive1.setPower(speed);
            leftDrive2.setPower(speed);
            rightDrive2.setPower(speed);

            // Display drive status for the driver.
            telemetry.addData("Target","%7d:%7d",newLeftTarget,newRightTarget);
            telemetry.addData("Actual","%7d:%7d:%7d:%7d",leftDrive1.getCurrentPosition(),leftDrive2.getCurrentPosition(),
                    rightDrive1.getCurrentPosition(),rightDrive2.getCurrentPosition());
            telemetry.addData("heading",new Func<String>(){
                @Override public String value(){
                    return formatAngle(angles.angleUnit,angles.firstAngle);
                }
            });

            // keep looping while we are still active, and BOTH motors are running.
            while(opModeIsActive()&&
                    (leftDrive1.isBusy()&&rightDrive1.isBusy())&&leftDrive2.isBusy()&&rightDrive2.isBusy()){

                // adjust relative speed based on heading error.
                error=getError(angle);
                steer=getSteer(error,P_DRIVE_COEFF);


                // if driving in reverse, the motor correction also needs to be reversed
                if(distance< 0)
                    steer*=-1.0;

                leftSpeed=speed-steer;
                rightSpeed=speed+steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max=Math.max(Math.abs(leftSpeed),Math.abs(rightSpeed));
                if(max>1.0)
                {
                    leftSpeed/=max;
                    rightSpeed/=max;
                }

                leftDrive1.setPower(leftSpeed);
                rightDrive1.setPower(rightSpeed);
                leftDrive2.setPower(leftSpeed);
                rightDrive2.setPower(rightSpeed);


                telemetry.update();

            }

            // Stop all motion;
            leftDrive1.setPower(0);
            rightDrive1.setPower(0);
            leftDrive2.setPower(0);
            rightDrive2.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void gyroStrafe(double speed,double distance,double angle){

        int newLeft1Target;
        int newRight1Target;
        int newLeft2Target;
        int newRight2Target;
        int moveCounts;
        double max1;
        double max2;
        double error;
        double steer;
        double left1Speed;
        double left2Speed;
        double right1Speed;
        double right2Speed;

        // Ensure that the opmode is still active
        if(opModeIsActive()){

            // Determine new target position, and pass to motor controller
            moveCounts=(int)(distance*COUNTS_PER_INCH_STRAFE);

            newLeft1Target=leftDrive1.getCurrentPosition()-moveCounts;
            newLeft2Target=leftDrive1.getCurrentPosition()+moveCounts;

            newRight1Target=rightDrive1.getCurrentPosition()+moveCounts;
            newRight2Target=rightDrive1.getCurrentPosition()-moveCounts;


            // Set Target and Turn On RUN_TO_POSITION
            leftDrive1.setTargetPosition(newLeft1Target);
            leftDrive2.setTargetPosition(newLeft2Target);

            rightDrive1.setTargetPosition(newRight1Target);
            rightDrive2.setTargetPosition(newRight2Target);

            leftDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed=Range.clip(Math.abs(speed),0.0,1.0);
            leftDrive1.setPower(speed);
            rightDrive1.setPower(speed);
            leftDrive2.setPower(speed);
            rightDrive2.setPower(speed);

            // Display drive status for the driver.
            telemetry.addData("Target","%7d:%7d",newLeft1Target,newRight1Target);
            telemetry.addData("Actual","%7d:%7d:%7d:%7d",leftDrive1.getCurrentPosition(),leftDrive2.getCurrentPosition(),
                    rightDrive1.getCurrentPosition(),rightDrive2.getCurrentPosition());
            telemetry.addData("heading",new Func<String>(){
                @Override public String value(){
                    return formatAngle(angles.angleUnit,angles.firstAngle);
                }
            });

            // keep looping while we are still active, and BOTH motors are running.
            while(opModeIsActive()&&
                    (leftDrive1.isBusy()&&rightDrive1.isBusy())&&leftDrive2.isBusy()&&rightDrive2.isBusy()){

                // adjust relative speed based on heading error.
                error=getError(angle);
                steer=getSteer(error,P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if(distance< 0)
                    steer*=-1.0;

                //leftSpeed = speed - steer;
                //rightSpeed = speed + steer;

                left1Speed=speed+steer;
                left2Speed=speed-steer;

                right1Speed=speed+steer;
                right2Speed=speed-steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max1=Math.max(Math.abs(left1Speed),Math.abs(right1Speed));
                max2=Math.max(Math.abs(left2Speed),Math.abs(right2Speed));

                if(max1>1.0)
                {
                    left1Speed/=max1;

                    right1Speed/=max1;

                }
                if(max2>1.0){
                    left2Speed/=max2;
                    right2Speed/=max2;
                }


                leftDrive1.setPower(left1Speed);
                leftDrive2.setPower(left2Speed);

                rightDrive1.setPower(right1Speed);
                rightDrive2.setPower(right2Speed);


                telemetry.update();

            }

            // Stop all motion;
            leftDrive1.setPower(0);
            rightDrive1.setPower(0);
            leftDrive2.setPower(0);
            rightDrive2.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void gyroTurn(double speed,double angle){

        // keep looping while we are still active, and not on heading.
        while(opModeIsActive()&&!onHeading(speed,angle,P_TURN_COEFF)){
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    public void gyroHold(double speed,double angle,double holdTime){

        ElapsedTime holdTimer=new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while(opModeIsActive()&&(holdTimer.time()<holdTime)){
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed,angle,P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        leftDrive1.setPower(0);
        rightDrive1.setPower(0);
        leftDrive2.setPower(0);
        rightDrive2.setPower(0);
    }


    /*
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    String formatAngle(AngleUnit angleUnit,double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit,angle));
    }

    String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(),"%.1f",AngleUnit.DEGREES.normalize(degrees));
    }

    public double getError(double targetAngle)
    {
        angles=imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);
        double robotError;


        // calculate error in -179 to +180 range  (
        robotError=targetAngle-angles.firstAngle;
        while(robotError>180)robotError-=360;
        while(robotError<=-180)robotError+=360;
        return robotError;
    }

    /*
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed,double angle,double PCoeff)
    {
        double error;
        double steer;
        boolean onTarget=false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error=getError(angle);

        if(Math.abs(error)<=HEADING_THRESHOLD){
            steer=0.0;
            leftSpeed=0.0;
            rightSpeed=0.0;
            onTarget=true;
        }
        else{
            steer=getSteer(error,PCoeff);
            rightSpeed=speed*steer;
            leftSpeed=-rightSpeed;
        }

        // Send desired speeds to motors.
        leftDrive1.setPower(leftSpeed);
        rightDrive1.setPower(rightSpeed);
        leftDrive2.setPower(leftSpeed);
        rightDrive2.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target","%5.2f",angle);
        telemetry.addData("Err/St","%5.2f/%5.2f",error,steer);
        telemetry.addData("Speed.","%5.2f:%5.2f",leftSpeed,rightSpeed);

        return onTarget;
    }

    /*
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error,double PCoeff){
        return Range.clip(error*PCoeff,-1,1);
    }
    public void printEncoders(){
        telemetry.addData("Actual","%7d:%7d:%7d:%7d",leftDrive1.getCurrentPosition(),leftDrive2.getCurrentPosition(),
                rightDrive1.getCurrentPosition(),rightDrive2.getCurrentPosition());

        telemetry.update();
    }


}