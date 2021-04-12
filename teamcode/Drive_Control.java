package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



@TeleOp(name="Demo Drive Controls", group="Iterative Opmode")
//@Disabled
public class Drive_Control extends LinearOpMode {

    // Declare vars (components)
    //motors are defined if taken from a top down view
    DcMotor intakeGreen;
    DcMotor intakeBlack;
    DcMotor shooterRight;
    DcMotor shooterLeft;

    DcMotor leftBack;
    DcMotor leftFront;
    DcMotor rightBack;
    DcMotor rightFront;
    CRServo servo;
    int dpishift = 1;
    double contPower;




    @Override
    public void runOpMode(){
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeGreen = hardwareMap.get(DcMotor.class, "intakeGreen");
        intakeGreen.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeGreen.setDirection(DcMotor.Direction.FORWARD);
        intakeGreen.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeGreen.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeBlack = hardwareMap.get(DcMotor.class, "intakeBlack");
        intakeBlack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeBlack.setDirection(DcMotor.Direction.REVERSE);
        intakeBlack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeBlack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterRight = hardwareMap.get(DcMotor.class, "ShooterRight");
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterRight.setDirection(DcMotor.Direction.REVERSE);
        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterLeft = hardwareMap.get(DcMotor.class, "ShooterLeft");
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servo = hardwareMap.get(CRServo.class, "servo");

        waitForStart();

        while(opModeIsActive()) {

            //intake
            if (gamepad2.right_bumper) {
                intakeGreen.setPower(1);
                intakeBlack.setPower(1);
            } else {
                intakeGreen.setPower(0);
                intakeBlack.setPower(0);
            }


            //shooter
            if (gamepad1.right_bumper) {
                shooterLeft.setPower(1);
                shooterRight.setPower(1);
            } else {
                shooterRight.setPower(0);
                shooterLeft.setPower(0);
            }

            //speed shift
            if (gamepad1.right_stick_x != 0) {

                if (gamepad1.left_bumper) {

                    dpishift = 2;
                }
                else {
                    dpishift = 1;
                }
            }

            if (gamepad1.left_bumper) {

                dpishift = 2;
            } else {

                dpishift = 1;
            }

            //servo
            contPower = gamepad1.left_trigger;
            servo.setPower(contPower);
            telemetry.addData("servoPower", contPower);


            //mecanum wheels
            leftBack.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + (gamepad1.right_stick_x)) / dpishift);

            rightBack.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - (gamepad1.right_stick_x)) / dpishift);

            leftFront.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - (gamepad1.right_stick_x)) / dpishift);

            rightFront.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + (gamepad1.right_stick_x)) / dpishift);

        }

    }

}
