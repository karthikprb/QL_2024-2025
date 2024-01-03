package org.firstinspires.ftc.teamcode.Components;

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
    public Caching_Servo intake_dropper;
    /*
    1. 0.33
    2. 0.27
    3.
    4.
    5. 0.09

     */

    private double pos =0;
    ElapsedTime time;
    Telemetry telemetry;

    boolean intakeToggle = false;
    private boolean outtake = false;

    public Intake(HardwareMap map, Telemetry telemetry){
        intake = new Caching_Motor(map, "intake");
        intake_dropper = new Caching_Servo(map, "intake_dropper");
        this.telemetry = telemetry;

        intake.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        time = new ElapsedTime();
        time.startTime();

    }

    public void intakeSet(double power){
        intake.setPower(power);
    }

    public void drop() {
        intake_dropper.setPosition(.1);
    }
    public void lift(){
        intake_dropper.setPosition(0.68);
    }

    public void intake(GamepadEx gamepadEx, GamepadEx gamepad2Ex, Telemetry telemetry){
        if(gamepadEx.isPress(GamepadEx.Control.right_bumper)){
            intakeToggle = !intakeToggle;
        }

        if(gamepadEx.isPress(GamepadEx.Control.right_trigger)){
            intakeToggle = false;
        }

        if(gamepad2Ex.isPress(GamepadEx.Control.dpad_up)){
            pos+=.01;
        }
        if(gamepad2Ex.isPress(GamepadEx.Control.dpad_down)){
            pos-=.01;
        }

        intake_dropper.setPosition(0.18+pos);

        if(intakeToggle){
            intake.setPower(1);
        }if(!intakeToggle) {
            intake.setPower(-gamepadEx.gamepad.right_trigger);
        }

        telemetry.addData("intake_dropper",intake_dropper.getPosition());

    }




    public void write(){
        intake.write();
        intake_dropper.write();
    }
}
