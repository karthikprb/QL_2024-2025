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
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@Config
public class Slides {
    public Caching_Motor vertSlide;

    public static double kp = 0.02;//0.02;
    public static double ki = 0.0;
    public static double kd = 0.0004;//0.0008;
    public static double gff = 0.25;//0.25;

    double sum = 0;
    ArrayList arr = new ArrayList();

    double slidePos;


    public static double pos;
    public static double heights;
    public PIDFController controller;

    Telemetry telemetry;
    ElapsedTime time;
    ElapsedTime secondTime;
    double startTime = 0;
    double retract = 0;
    double extend = 0;

    boolean manualOperateToggle = true;

    public static boolean hangToggle = false;

    public TouchSensor touchSensor;

    public enum STATE{
        AUTO,
        MANUAL,
    }

    public STATE mRobotState = STATE.AUTO;


    public Slides(HardwareMap map, Telemetry telemetry){
        this.telemetry = telemetry;

        vertSlide = new Caching_Motor(map, "lslide");
        controller = new PIDFController(new PIDCoefficients(kp, ki, kd));
        reset();

    }

    public void write(){
        vertSlide.write();
    }

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
        vertSlide.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertSlide.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public double getPosition(){
        double slidePos = (vertSlide.motor.getCurrentPosition());
        telemetry.addData("Slide Position", slidePos);
        return Math.abs(slidePos);
    }

    public void setPower(double power){
        vertSlide.setPower(power);
    }

    public void operate(GamepadEx gamepad1, GamepadEx gamepad2){
    setPower(gamepad1.gamepad.left_stick_y*-1);


    }
}
