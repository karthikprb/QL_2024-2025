package org.firstinspires.ftc.teamcode.Components;

import android.service.vr.VrListenerService;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import org.checkerframework.checker.units.qual.C;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Motor;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@Config
public class Slides {
    public Caching_Motor lSlide;
    public Caching_Motor rSlide;
    public boolean down = false;
    public double inp;
    public static double kp = 0.03;//0.02;
    public static double ki = 0.0;
    public static double kd = 0.0004;//0.0008;
    public static double gff = 0.25;//0.25;
    double sum = 0;
    ArrayList arr = new ArrayList();

    double slidePos;

    public V4B_Arm v4B_arm;

    public static double pos;
    public static double height;
    public STATE mRobotState;
    public PIDFController controller;
    Telemetry telemetry;
    ElapsedTime time;
    ElapsedTime secondTime;
    double startTime = 0;

    //DigitalChannel digitalTouch;

    public enum STATE{
        AUTO,
        MANUAL,
    }


    public Slides(HardwareMap map, Telemetry telemetry){
        this.telemetry = telemetry;
        rSlide = new Caching_Motor(map, "rslide");
        lSlide = new Caching_Motor(map, "lslide");
        controller = new PIDFController(new PIDCoefficients(kp, ki, kd));
        reset();
        /*
        digitalTouch = map.get(DigitalChannel.class, "sensor_digital");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

         */

        time = new ElapsedTime();
        secondTime =new ElapsedTime();
        secondTime.startTime();
        time.startTime();
        setBrake();
    }

    public void write(){
        rSlide.write();
        lSlide.write();
    }
    /*
    public boolean isDown(){
        return !digitalTouch.getState();
    }

     */

    public void setPosition(double target){
        controller.setTargetPosition(target);
        setPower(controller.update(getPosition()));

        telemetry.addData("Target", target);
        telemetry.addData("Error", controller.getLastError());
    }

    public void setPosition(double target, double lowerBound, double upperBound){
        controller.setTargetPosition(target);
        setPower(Range.clip(controller.update(getPosition()), lowerBound, upperBound));

        telemetry.addData("Target", target);
        telemetry.addData("Error", controller.getLastError());
    }

    public void reset(){
        lSlide.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lSlide.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rSlide.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rSlide.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getPosition(){
        double slidePos = (rSlide.motor.getCurrentPosition() - lSlide.motor.getCurrentPosition())/2.0;
        telemetry.addData("Slide Position", slidePos);
        return Math.abs(slidePos);
    }

    public void setCoast(){
        rSlide.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lSlide.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setBrake(){
        rSlide.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lSlide.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double power){
        rSlide.setPower(power);
        lSlide.setPower(-power);
    }


    public void operate(GamepadEx gamepad1, GamepadEx gamepad2){
        /*switch(mRobotState){
            case AUTO:
                if(gamepad2.isPress(GamepadEx.Control.dpad_up)){
                    height++;
                }
                if(gamepad2.isPress(GamepadEx.Control.dpad_down)){
                    height--;
                }

                if(gamepad2.isPress(GamepadEx.Control.y)){
                    height = 10;
                }
                if(gamepad2.isPress(GamepadEx.Control.b)){
                    height = 5;
                }
                if(gamepad2.isPress(GamepadEx.Control.a)){
                    height = 0;
                }

                if(height == 1) {

                }if(height == 2){

                }if(height == 3) {

                }if(height == 4){

                }if(height == 5){

                }if(height == 6){

                }if(height == 7){

                }if(height == 8){

                }if(height == 9){

                }if(height == 10) {

                }if(height == 11){

                }

                break;
            case MANUAL:
            */

                secondTime.reset();


                if(gamepad2.gamepad.left_stick_y>0){
                    setPower(gamepad2.gamepad.left_stick_y);

                    sum+=gamepad2.gamepad.left_stick_y*secondTime.time();

                }else if(gamepad1.gamepad.left_stick_y<0){
                    setPower(gamepad2.gamepad.left_stick_y*0.4);
                    sum+=gamepad2.gamepad.left_stick_y*0.4*secondTime.time();
                }else if(gamepad1.gamepad.left_stick_y<0){
                    setPower(0);
                }/*else if(gamepad2.isPress(GamepadEx.Control.b)){
                    down = !down;
                }*/else{
                setPower(0);
                }

                /*
                if(down){
                    setPower(-.5);
                    if(isDown()){
                        down = false;
                    }
                }

                 */








                //setPosition(posi);




                

                /*
                if(v4B_arm.duoToggle){
                    if(v4B_arm.grabberToggle == 4){
                        setPower(-0.2);
                    }
                }
                if(!v4B_arm.duoToggle){
                    if(v4B_arm.grabberToggle == 5){
                        setPower(-0.2);
                    }
                }
                 */
        /*
                break;
        }
         */



        //telemetry.addData("Slide Position: ", getPosition());
        //telemetry.addData("Left", lSlide.motor.getCurrentPosition());
        //telemetry.addData("Right", rSlide.motor.getCurrentPosition());
        telemetry.addData("slides pos",sum);
        telemetry.addData("Right Slide Power: ", rSlide.motor.getPower());
        telemetry.addData("Left Slide Power: ", lSlide.motor.getPower());
        telemetry.addData("Right Stick", gamepad2.gamepad.left_stick_y);


    }
}
