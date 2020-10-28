package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto_Frame_2", group="Pushbot")
//@Disabled
public class PushbotAutoDriveByTime_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();


    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    public void step1(){
        // Step 1:  Drive forward for 3.5 seconds
        robot.leftfront.setPower(FORWARD_SPEED);
        robot.leftback.setPower(FORWARD_SPEED);
        robot.rightfront.setPower(FORWARD_SPEED);
        robot.rightback.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.5)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Spin right for .5 seconds
        robot.leftfront.setPower(TURN_SPEED);
        robot.leftback.setPower(TURN_SPEED);
        robot.rightfront.setPower(-TURN_SPEED);
        robot.rightback.setPower(-TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 3:  Drive forward for .2 seconds
        robot.leftfront.setPower(FORWARD_SPEED);
        robot.leftback.setPower(FORWARD_SPEED);
        robot.rightfront.setPower(FORWARD_SPEED);
        robot.rightback.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.2)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);

    }
    }

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way
        step1();
        //step2();
        public void step2(){

            // Step 1:  Drive backward for .2 seconds
            robot.leftfront.setPower(-FORWARD_SPEED);
            robot.leftback.setPower(-FORWARD_SPEED);
            robot.rightfront.setPower(-FORWARD_SPEED);
            robot.rightback.setPower(-FORWARD_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.2)) {
                telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            // Step 2:  Spin left for .5 seconds
            robot.leftfront.setPower(-TURN_SPEED);
            robot.leftback.setPower(-TURN_SPEED);
            robot.rightfront.setPower(TURN_SPEED);
            robot.rightback.setPower(TURN_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            // Step 3:  Drive backward for .8 seconds
            robot.leftfront.setPower(-FORWARD_SPEED);
            robot.leftback.setPower(-FORWARD_SPEED);
            robot.rightfront.setPower(-FORWARD_SPEED);
            robot.rightback.setPower(-FORWARD_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.8)) {
                telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);
        }


}