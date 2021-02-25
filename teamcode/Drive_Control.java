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
    DcMotor intakeGreen;
    DcMotor intakeBlack;
    DcMotor shooterRight;
    DcMotor shooterLeft;

    DcMotor leftBack;
    DcMotor leftFront;
    DcMotor rightBack;
    DcMotor rightFront;
    int dpishift = 1;


    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.leftFront.setTargetPosition(newLeftTarget);
            robot.rightFront.setTargetPosition(newRightTarget);
            robot.leftBack.setTargetPosition(newLeftTarget);
            robot.rightBack.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFront.setPower(Math.abs(speed));
            robot.rightFront.setPower(Math.abs(speed*DRIFT_VARIABLE));
            robot.leftBack.setPower(Math.abs(speed));
            robot.rightBack.setPower(Math.abs(speed*DRIFT_VARIABLE));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFront.isBusy() && robot.rightFront.isBusy() && robot.leftBack.isBusy() && robot.rightBack.isBusy())) {
                telemetry.addData("Running","");
                // Display it for the driver.
                telemetry.update();
            }

            // Stop all motion;
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }


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
        intakeGreen.setDirection(DcMotor.Direction.REVERSE);
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




        //runtime.reset();

        waitForStart();

        while(opModeIsActive()) {

            if (gamepad2.right_bumper) {
                intakeGreen.setPower(1);
                intakeBlack.setPower(1);
            } else {
                intakeGreen.setPower(0);
                intakeBlack.setPower(0);
            }


            if (gamepad1.right_bumper) {
                shooterLeft.setPower(1);
                shooterRight.setPower(1);
            } else {
                shooterRight.setPower(0);
                shooterLeft.setPower(0);
            }

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


            leftBack.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + (gamepad1.right_stick_x)) / dpishift);

            rightBack.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - (gamepad1.right_stick_x)) / dpishift);

            leftFront.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - (gamepad1.right_stick_x)) / dpishift);

            rightFront.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + (gamepad1.right_stick_x)) / dpishift);

        }

    }

}
