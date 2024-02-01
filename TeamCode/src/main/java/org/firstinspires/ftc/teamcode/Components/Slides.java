package org.firstinspires.ftc.teamcode.Components;

import android.service.vr.VrListenerService;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import org.checkerframework.checker.units.qual.C;

import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Motor;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@Config
public class Slides {
    public Caching_Motor lSlide;
    public Caching_Motor rSlide;
    public boolean down = false;
    public double inp;
    public static double kp = 0.02;//0.02;
    public static double ki = 0.0;
    public static double kd = 0.0004;//0.0008;
    public static double gff = 0.25;//0.25;

    double sum = 0;
    ArrayList arr = new ArrayList();

    double slidePos;


    public static double pos;
    public static double height;
    public PIDFController controller;

    Telemetry telemetry;
    ElapsedTime time;
    ElapsedTime secondTime;
    double startTime = 0;

    boolean manualOperateToggle = true;

    boolean hangToggle = false;

    public TouchSensor touchSensor;

    public enum STATE{
        AUTO,
        MANUAL,
    }

    public STATE mRobotState = STATE.AUTO;


    public Slides(HardwareMap map, Telemetry telemetry){
        this.telemetry = telemetry;
        rSlide = new Caching_Motor(map, "rslide");
        lSlide = new Caching_Motor(map, "lslide");
        controller = new PIDFController(new PIDCoefficients(kp, ki, kd));
        reset();

        touchSensor = map.get(TouchSensor.class, "sensor_digital");


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

        if(manualOperateToggle) {
            switch (mRobotState) {
                case AUTO:
                    if (gamepad2.gamepad.left_stick_y > 0.03 || gamepad2.gamepad.left_stick_y < -0.03) {
                        mRobotState = STATE.MANUAL;
                    } else {
                        if (V4B_Arm.grabberToggle == 2 || V4B_Arm.grabberToggle == 3) {
                            setPosition(400);
                        } else {
                            if (touchSensor.isPressed()) {
                                setPower(0.0);
                            } else {
                                setPower(-0.7);
                            }
                        }
                    }
                    break;
                case MANUAL:
                    if (gamepad1.gamepad.left_bumper || gamepad2.gamepad.left_bumper) {
                        if(V4B_Arm.grabberToggle == 4){
                            mRobotState = STATE.AUTO;
                        } else {
                            setPower(0.4);
                        }
                    } else {
                        if (touchSensor.isPressed()) {
                            if (gamepad2.gamepad.left_stick_y > 0) {
                                setPower(gamepad2.gamepad.left_stick_y);

                            } else {
                                setPower(0.0);
                            }
                        } else {
                            if (hangToggle) {
                                if (gamepad2.gamepad.left_stick_y > 0.03) {
                                    setPower(gamepad2.gamepad.left_stick_y);


                                } else if (gamepad1.gamepad.left_stick_y < 0.03) {
                                    setPower(gamepad2.gamepad.left_stick_y);
                                } else {
                                    setPower(0.0);
                                }
                            } else {
                                if (gamepad2.gamepad.left_stick_y > 0.03) {
                                    setPower(gamepad2.gamepad.left_stick_y);


                                } else if (gamepad1.gamepad.left_stick_y < 0.03) {
                                    setPower(gamepad2.gamepad.left_stick_y * 0.6);
                                } else {
                                    setPower(0.0);
                                }
                            }
                        }
                    }
                    break;
            }
        } else {
            if (touchSensor.isPressed()) {
                if (gamepad2.gamepad.left_stick_y > 0) {
                    setPower(gamepad2.gamepad.left_stick_y);

                } else {
                    setPower(0.0);
                }
            } else {
                if (gamepad2.gamepad.left_stick_y > 0.03) {
                    setPower(gamepad2.gamepad.left_stick_y);


                } else if (gamepad1.gamepad.left_stick_y < 0.03) {
                    setPower(gamepad2.gamepad.left_stick_y);
                } else {
                    setPower(0.0);
                }
            }
        }

        if(gamepad2.isPress(GamepadEx.Control.b)){
            hangToggle = !hangToggle;
        }


        if(gamepad2.isPress(GamepadEx.Control.start)){
            manualOperateToggle = !manualOperateToggle;
        }

        if (gamepad1.isPress(GamepadEx.Control.left_bumper) || gamepad2.isPress(GamepadEx.Control.left_bumper)){
            time.reset();
        }



        telemetry.addData("Slide Position: ", getPosition());
        telemetry.addData("Left", lSlide.motor.getCurrentPosition());
        telemetry.addData("Right", rSlide.motor.getCurrentPosition());
        telemetry.addData("slides pos",sum);
        telemetry.addData("Right Slide Power: ", rSlide.motor.getPower());
        telemetry.addData("Left Slide Power: ", lSlide.motor.getPower());
        telemetry.addData("Right Stick", gamepad2.gamepad.left_stick_y);
        telemetry.addData("Touch Sensor", touchSensor.isPressed());
        telemetry.addData("Touch Value", touchSensor.getValue());
        telemetry.addData("Case", mRobotState);
        telemetry.addData("GrabberToggle", V4B_Arm.grabberToggle);
        telemetry.addData("HangToggle", hangToggle);
        telemetry.addData("Time", time);
    }
}
