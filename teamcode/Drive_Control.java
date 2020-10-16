package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Demo Drive Controls", group="Iterative Opmode")
//@Disabled

public class Drive_Control extends OpMode {

    // Declare vars (components)
    //motors are defined if taken from a top down view
    //Xavier was here

    DcMotor leftBack;
    DcMotor leftFront;
    DcMotor rightBack;
    DcMotor rightFront;


    @Override
    public void runOpMode(){
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");

        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");

        rightFront = hardwareMap.get(DcMotor.class, "rightFront");


        runtime.reset();

        waitForStart();

        while(opModeIsActive()) {
            double gamma = 1.5; //meant to counteract imperfect strafing, driver preference
            double y = -gamepad1.left_stick_y; //reversed
            double x = gamepad1.right_stick._x * gamma;
            double rx = gamepad1.right_stick_x;

            leftFront.setPower(y + x + rx);
            leftBack.setPower(y - x + rx);
            rightFront.setPower(y - x - rx); 
            rightBack.setPower(y + x - rx);

        }

    }

}