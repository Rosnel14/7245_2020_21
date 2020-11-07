package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



@TeleOp(name="Demo Drive Controls", group="Iterative Opmode")
//@Disabled
public class Drive_Control extends LinearOpMode {

    // Declare vars (components)
    //motors are defined if taken from a top down view

    DcMotor leftBack;
    DcMotor leftFront;
    DcMotor rightBack;
    DcMotor rightFront;
    int dpishift = 1;


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


        //runtime.reset();

        waitForStart();

        while(opModeIsActive()) {

            if (gamepad1.right_stick_x != 0) {

                if (gamepad1.left_bumper) {

                    dpishift = 10;
                }
                else {
                    dpishift = 2;
                }
            }

            if (gamepad1.left_bumper) {

                dpishift = 5;
            } else {

                dpishift = 1;
            }


            leftBack.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + (gamepad1.right_stick_x)) / dpishift);

            rightBack.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - (gamepad1.right_stick_x)) / dpishift);

            leftFront.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - (gamepad1.right_stick_x)) / dpishift);

            rightFront.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + (gamepad1.right_stick_x)) / dpishift);

        }

    }

}