package org.firstinspires.ftc.teamcode;

import android.widget.Spinner;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 *import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 *import com.qualcomm.robotcore.hardware.DcMotor;
 *
 *
 */

/**
 *Notes For this TeleOp Code. This code is for Comp and all proggramers should review over this
 *code and understand this code for the possibility that a question may be asked related to TeleOp and
 *you should be able to explain in good detail everything in this code.
 *11/16/17-> Changed all gamepad's in code to correct gamepad (i.e some gamepad1's to gamepad2)
 ***11/18/17-> Competition Notes below
 *Notes-> Autonomous is incorrect, Not much was wrong from a software sandpoint but hardware issues were fixed
 *Autonomous issues included: Incorrect spinning causing us to move out of destination,
 *To much time on the down motion of the clamp and arm.
 *These issues are still not resolved
 * Recomendation for autonomous issues(Not Offical):Fine tune the timer on the clamp
 * Fine tune the movements and LOWER the TIME OF MOVEMENT in autonomous.
 * List of issues at Comp(1)-> https://docs.google.com/a/stjoebears.com/spreadsheets/d/1r_liipKBU7GHfONdxq9E6d4f7zikcCuXwDL2bsQfwm0/edit?usp=sharing
 *G-Sheet of time VS Heading for autonomous -> https://docs.google.com/a/stjoebears.com/spreadsheets/d/1pqv0iN94fFd5KvX1YIWP7z39HgpURXsscn0zPujs1q4/edit?usp=sharing
*/
@TeleOp(name="Simple Mecanum Drive", group="TeleOp")

public class teleOpSimpleMecanum extends OpMode {

    private MecanumDrive mecanumDrive = new MecanumDrive();
    private CarouselSpinner spinner = new CarouselSpinner();

    double forward;
    double clockwise;
    double right;
    double k;
    double power0;
    double power1;
    double power2;
    double power3;
    double max;

    boolean aPressedInit;
    boolean aPrev = false;
    boolean bPrev = false;
    boolean yprev = false;
    protected int driveStyle=1;
    private double[] distances;

    HardwareChassisBot robot = new HardwareChassisBot();

    @Override
    // Code to run ONCE when the operators Initializes the robot
    public void init(){

        mecanumDrive.init(hardwareMap);
        spinner.init(hardwareMap);
    }

    // Code to run REPEATEDLY until the driver hits PLAY
    public void init_loop() {
        //This code should be executed in a loop while waiting for the start button to be pressed.
        //This is where we'll ask the driver what type of drive they would prefer

        if (gamepad1.a && !aPressedInit) {
            // The User is pressing "A" to change the Drive Style
            if(driveStyle == 3){
                // Revert to 1
                driveStyle=1;
                telemetry.addLine("Drive Style: JoeBots Mecanum");
            } else if (driveStyle == 1) {
                // increase to Drive Style 2
                driveStyle=2;
                telemetry.addLine("Drive Style: Halo Standard");
            } else if (driveStyle == 2) {
                // Increase to DriveStyle 3
                driveStyle=3;
                telemetry.addLine("Drive Style: Halo Southpaw");
            }
        }
        aPressedInit = gamepad1.a;
        telemetry.addLine("Press the 'A' button to change Drive Style");
        if (driveStyle == 1) {
            telemetry.addLine("Drive Style: JoeBots Mecanum");
        } else if (driveStyle == 2) {
            telemetry.addLine("Drive Style: Halo Standard");
        } else if (driveStyle == 3) {
            telemetry.addLine("Drive Style: Halo Southpaw");
        }
        telemetry.addData("Drive Style: ", driveStyle);
        telemetry.update();

    }

    @Override
    public void loop() {

        double forward;
        double strafe;
        double rotate;

        // Read the driveStyle variable and set the drive components appropriately.
        switch (driveStyle) {

            case 1:
                // JoeBots Mecanum
                forward = gamepad1.left_stick_y * -1;
                strafe = gamepad1.right_trigger - gamepad1.left_trigger;
                rotate = gamepad1.right_stick_x;
                break;

            case 2:
                // Halo Standard Drive
                forward = gamepad1.left_stick_y * -1;
                strafe = gamepad1.left_stick_x;
                rotate = gamepad1.right_stick_x;
                break;

            case 3:
                // Halo Southpaw
                forward = gamepad1.right_stick_y * -1;
                strafe = gamepad1.right_stick_x;
                rotate = gamepad1.left_stick_x;
                break;

            default:
                throw new IllegalStateException("Unexpected value: " + driveStyle);
        }

        if (gamepad2.a && !aPrev) {
            spinner.Toggle();
        }
        aPrev = gamepad2.a;

        if (gamepad2.b && !bPrev){
            spinner.reverse();
        }
        bPrev = gamepad2.b;

        if (gamepad2.y && !yprev){
            spinner.startRamp();
        }
        yprev = gamepad2.y;


        if (gamepad2.right_trigger > 0){
            spinner.ManualDrive(gamepad2.right_trigger);
        }

        /*
        if(gamepad2.right_bumper) {
            spinner.setDriverOverride();
            spinner.powerOn();
        } else {
            spinner.disableDriveOverride();
        }
        */

        mecanumDrive.driveMecanum(forward, strafe, rotate);
        spinner.spinnerController();
        distances = mecanumDrive.getDistanceCm();
        telemetry.addData("distance fwd", distances[0]);
        telemetry.addData("distance right", distances[1]);


    }

}