package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class CarouselSpinner {

    //declare motors
    public DcMotor Spinner;

    //Hardware map stuff
    HardwareMap hwMap;
    private ElapsedTime period = new ElapsedTime();

    private boolean SpinnerRunning = false;
    private boolean driverOverride = false;

    public double currentPower = 0;
    public double spinnerMaxPower = 0.3;
    boolean rampingUp = false;
    boolean Reversing = false;

    private static final double INCREMENT = 0.05;
    private static final double CYCLE = 50;

    private ElapsedTime rampTime = new ElapsedTime();
    private double rampCurrTime = 0;



    //constants
    public static final String ConfigNameBlue ="Spinner";

    //constructor
    public CarouselSpinner(){

    }

    //init
    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;
        //init motors
        Spinner = hwMap.dcMotor.get(ConfigNameBlue);

        //motor directions
        Spinner.setDirection(DcMotor.Direction.FORWARD);

        //stop motors
        stopMotor();
        //run without encoders
        Spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    ////=======================

    //stop
    public void stopMotor(){
        currentPower = 0;
    }

    public void setDriverOverride(){
        driverOverride=true;
        currentPower = 1;
    }
    public void disableDriveOverride(){
        driverOverride=false;
        currentPower = 0;
    }

    //ramp up
    public void startRamp(){
        rampingUp = true;
    }
    public void rampController(){
        if(rampTime.milliseconds()-rampCurrTime >= CYCLE) {
            currentPower += INCREMENT;
            }
        if(currentPower>=spinnerMaxPower){
            rampingUp = false;
        }

    }
    //reverse
    public void reverse() {

        if (Spinner.getDirection() == DcMotor.Direction.FORWARD) {
            // The motor is currently running forward
            Spinner.setDirection(DcMotor.Direction.REVERSE);
        } else {
            Spinner.setDirection(DcMotor.Direction.FORWARD);
        }

    }
    //set speed
    public void ManualDrive(double newSpeed){
        currentPower = newSpeed;
    }

    //max power
    public void powerOn(){
        if(!driverOverride) {
            currentPower = spinnerMaxPower;
        }
    }
    public void Toggle(){
        if (SpinnerRunning) {
            currentPower = 0;
        } else {
            currentPower = spinnerMaxPower;
        }

    }

    public void spinnerController(){
        //sets a variable to control the current action of the motor
        if(currentPower>0 || currentPower<0){
            //the motor is running. Check if ramping
            if(rampingUp){
                rampController();
            }

            SpinnerRunning = true;
        } else{
            SpinnerRunning = false;
            currentPower = 0;
        }
        Spinner.setPower(currentPower);
    }
}
