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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="ArmTeleop", group="Iterative Opmode")
//@Disabled
public class Arm_Teleop extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor left1 = null;
    private DcMotor right1 = null;
    private DcMotor left2 = null;
    private DcMotor right2 = null;

    private DcMotor arm = null;




    //Gyro Setup
    BNO055IMU imu;
    Orientation angles;

    //Other setup
    boolean pressed;
    public static double kp = 0.02;
    double modifier = 0.6;

    @Override
    public void runOpMode() {

        //Drive Train
        left1 = hardwareMap.get(DcMotor.class, "left1");
        right1 = hardwareMap.get(DcMotor.class, "right1");
        left2 = hardwareMap.get(DcMotor.class, "left2");
        right2 = hardwareMap.get(DcMotor.class, "right2");
        arm = hardwareMap.get(DcMotor.class,"arm");


        left1.setDirection(DcMotor.Direction.FORWARD);
        right2.setDirection(DcMotor.Direction.REVERSE);
        left2.setDirection(DcMotor.Direction.FORWARD);
        right1.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.FORWARD);


        //Gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit= BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            //Drive Train
            double leftPower;
            double rightPower;

            double drive = modifier * -gamepad1.left_stick_y;
            double turn = modifier * gamepad1.right_stick_x;
            leftPower = Range.clip(drive + turn, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0);

            //Strafe Code
            if (gamepad1.left_bumper) {
                pressed = true;
                StrafeLeft(angles.firstAngle);
            }
            else if (gamepad1.right_bumper) {
                pressed = true;
                StrafeRight(angles.firstAngle);
            }
            else {
                pressed = false;

                left1.setPower(leftPower);
                right1.setPower(rightPower);
                left2.setPower(leftPower);
                right2.setPower(rightPower);

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }

            //arm code
            if (gamepad1.left_stick_button) {
                pressed = true;
                LiftArm();
            }
            else if (gamepad1.right_stick_button) {
                pressed = true;
                LowerArm();
            } else{

            }


        }
    }

    private void LiftArm() {
        arm.setPower(0.5);
    }

    private void LowerArm() {
        arm.setPower(-0.5);
    }



    //Movement
    private void StrafeLeft(double angle) {
        double max1;
        double max2;
        double error;
        double steer;
        double left1Speed;
        double left2Speed;
        double right1Speed;
        double right2Speed;

        while (opModeIsActive() && pressed) {

            if (!gamepad1.left_bumper)
                pressed = false;

            error = getErrorAngle(angle);
            steer = getSteer(error, kp);

            left1Speed = modifier - steer;
            left2Speed = -modifier - steer;

            right1Speed = -modifier + steer;
            right2Speed = modifier + steer;

            // Normalize speeds if either one exceeds +/- 1.0;
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
        }
        left1.setPower(0);
        right1.setPower(0);
        left2.setPower(0);
        right2.setPower(0);
    }
    private void StrafeRight(double angle) {

        double max1;
        double max2;
        double error;
        double steer;
        double left1Speed;
        double left2Speed;
        double right1Speed;
        double right2Speed;

        while (opModeIsActive() && pressed) {

            if (!gamepad1.right_bumper)
                pressed = false;

            error = getErrorAngle(angle);
            steer = getSteer(error, kp);

            left1Speed = -modifier - steer;
            left2Speed = modifier - steer;

            right1Speed = modifier + steer;
            right2Speed = -modifier + steer;

            // Normalize speeds if either one exceeds +/- 1.0;
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
        }
        left1.setPower(0);
        right1.setPower(0);
        left2.setPower(0);
        right2.setPower(0);
    }

    //Gyro functions
    private double getErrorAngle(double targetAngle) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
    private double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}