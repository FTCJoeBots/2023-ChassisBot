package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class validate extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motor0;
        DcMotor motor1;
        DcMotor motor2;
        DcMotor motor3;

        motor0 = hardwareMap.get(DcMotor.class, "leftFront");
        motor1 = hardwareMap.get(DcMotor.class,"rightFront");
        motor2 = hardwareMap.get(DcMotor.class,"leftRear");
        motor3 = hardwareMap.get(DcMotor.class,"rightRear");

        motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (!isStopRequested()) {






            telemetry.addData("motor0 encoder:",motor0.getCurrentPosition());
            telemetry.addData("motor1 encoder:",motor1.getCurrentPosition());
            telemetry.addData("motor2 encoder:",motor2.getCurrentPosition());
            telemetry.addData("motor3 encoder:",motor3.getCurrentPosition());
            telemetry.update();
        }
    }
}