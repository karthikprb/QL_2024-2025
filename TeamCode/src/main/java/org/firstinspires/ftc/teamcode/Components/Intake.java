package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Motor;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

public class Intake {
    Caching_Motor intake;
<<<<<<< HEAD
    double pos = .945;
=======
    double pos = .76;
>>>>>>> 5ff0dcc7faacc123531080944104cd678be9759e
    public Caching_Servo intake_dropper;
    /*
    1. 0.33
    2. 0.27
    3.
    4.
    5. 0.09

     */


    private double intakeHeight = 0.0;
    ElapsedTime time;
    Telemetry telemetry;

    boolean intakeToggle = false;

    double intakeValue = 0;

    boolean slideSpeedToggle = false;
    private boolean outtake = false;

    public Intake(HardwareMap map, Telemetry telemetry){
        intake = new Caching_Motor(map, "intake");
        intake_dropper = new Caching_Servo(map, "intake_dropper");
        this.telemetry = telemetry;
        intake.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        time = new ElapsedTime();
        time.startTime();
        lift();
        write();
    }

    public void intakeSet(double power){
        intake.setPower(power);
    }

   public void drop() {
        intake_dropper.setPosition(1);
    }
    public void lift(){
        intake_dropper.setPosition(0.07);
    }


    public void outtakeDeposit(){
        intake.setPower(-0.75);
    }

    public void stop(){
        intake.setPower(0);
    }

    public void intake(GamepadEx gamepadEx, GamepadEx gamepad2Ex, Telemetry telemetry){

        if(gamepad2Ex.isPress(GamepadEx.Control.b)){
            slideSpeedToggle = !slideSpeedToggle;
        }


        if(slideSpeedToggle){
            gamepad2Ex.gamepad.rumble(100);
            lift();
        } else {
            if(intakeValue == 1){
                intake_dropper.setPosition(0.87);
            } else {
                intake_dropper.setPosition(pos);

            }
        }



        if(gamepadEx.isPress(GamepadEx.Control.right_bumper)){
            intakeToggle = !intakeToggle;
        }

        if(gamepadEx.isPress(GamepadEx.Control.right_trigger)){
            intakeToggle = false;
        }


<<<<<<< HEAD
        if(gamepad2Ex.isPress(GamepadEx.Control.dpad_right)){
            intakeValue = 1;
        }
        if(gamepad2Ex.isPress(GamepadEx.Control.dpad_left)){
            intakeValue = 0;
=======

        intake_dropper.setPosition(pos);

        if(gamepad2Ex.isPress(GamepadEx.Control.dpad_right)){
            pos -= .02;
        }
        if(gamepad2Ex.isPress(GamepadEx.Control.dpad_left)){
            pos += .02;
>>>>>>> 5ff0dcc7faacc123531080944104cd678be9759e
        }

        if(intakeToggle){
            if(intakeHeight == 0){
                intake.setPower(1.0);
            } else if (intakeHeight == 1){
                intake.setPower(1.0);
            } else if(intakeHeight == 2){
                intake.setPower(1.0);
            }else if(intakeHeight == 3){
                intake.setPower(1.0);
            } else if(intakeHeight == 4){
                intake.setPower(1.0);
            } else if(intakeHeight == 5){
                intake.setPower(1.0);
            }
        }

        if(!intakeToggle) {
            intake.setPower(-gamepadEx.gamepad.right_trigger);
        }

       telemetry.addData("intake_dropper",intake_dropper.getPosition());

    }




    public void write(){
        intake.write();
        intake_dropper.write();
    }
}
