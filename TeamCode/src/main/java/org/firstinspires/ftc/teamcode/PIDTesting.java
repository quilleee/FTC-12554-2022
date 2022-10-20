
package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


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

@TeleOp(name="PIDTesting", group="CommandOpMode")
//@Disabled
public class PIDTesting extends CommandOpMode {

    public DcMotor  driveOne = null;
    public DcMotor  driveTwo = null;
    public DcMotor  driveThree = null;
    public DcMotor  driveFour = null;

    public Servo leftClaw = null;
    public Servo rightClaw = null;

    double clawOffset = 0;

    public static final double MID_SERVO   =  0.5 ;

    // Creates a PIDFController with gains kP, kI, kD, and kF
    // help i dont know what to do
    PIDFController pidf = new PIDFController(kP, kI, kD, kF);

    /*
     * Here are the constructors for the other controllers
     */
    PIDController pid = new PIDController(kP, kI, kD);
    PDController pd = new PDController(kP, kD);
    PController p = new PController(kP);

    // set our gains to some value
pidf.setP(0.37);
pidf.setI(0.05);
pidf.setD(1.02);

// get our gain constants
    kP = pidf.getP();
    kI = pidf.getI();
    kD = pidf.getD();

// set all gains
pidf.setPIDF(kP, KI, kD, 0.7);

    // get all gain coefficients
    double[] coeffs = pidf.getCoefficients();
    kP = coeffs[0];
    kI = coeffs[1];
    kD = coeffs[2];
    kF = coeffs[3];

    @Override
    public void initialize(){
        waitForStart();
        double left;
        double right;
        double drive;
        double turn;
        double max;

        driveOne  = hardwareMap.get(DcMotor.class, "drive_one");
        driveTwo = hardwareMap.get(DcMotor.class, "drive_two");
        driveThree = hardwareMap.get(DcMotor.class, "drive_three");
        driveFour = hardwareMap.get(DcMotor.class, "drive_four");

        driveOne.setDirection(DcMotor.Direction.FORWARD);
        driveTwo.setDirection(DcMotor.Direction.REVERSE);
        driveThree.setDirection(DcMotor.Direction.FORWARD);
        driveFour.setDirection(DcMotor.Direction.REVERSE);

        leftClaw  = hardwareMap.get(Servo.class, "left_hand");
        rightClaw = hardwareMap.get(Servo.class, "right_hand");
        leftClaw.setPosition(MID_SERVO);
        rightClaw.setPosition(MID_SERVO);

        telemetry.addData(">", "The robot is ready,, press play");    //
        telemetry.update();

        while (opModeIsActive()) {
            drive = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;

            left = drive + turn;        //turning but smoother
            right = drive - turn;

            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) //make sure it doesn't go over 1.0
            {
                left /= max;
                right /= max;
            }

            driveOne.setPower(left);
            driveTwo.setPower(right);
            driveThree.setPower(left);
            driveFour.setPower(right);

            if (gamepad1.a) {
                leftClaw.setPosition(0.3);
                rightClaw.setPosition(0.7);
            }
            else if (gamepad1.b) {
                leftClaw.setPosition(0.6);
                rightClaw.setPosition(0.4);
            }

            clawOffset = Range.clip(clawOffset, -0.5, 0.5);
            leftClaw.setPosition(MID_SERVO + clawOffset);
            rightClaw.setPosition(MID_SERVO - clawOffset);

            telemetry.addData("claw",  "Offset = %.2f", clawOffset);
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.update();

            sleep(50);

        }

    }

}