package org.firstinspires.ftc.teamcode.Components;

import android.app.admin.DevicePolicyManager;
import android.hardware.Sensor;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.C;
import org.checkerframework.checker.units.qual.Time;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

public class V4B_Arm {

    Caching_Servo rightArm;
    Caching_Servo leftArm;
    Caching_Servo leftGrab;
    Caching_Servo rightGrab;

    ElapsedTime time = new ElapsedTime();
    ElapsedTime secondTime = new ElapsedTime();
    public double grabberToggle = 0;
    boolean duoToggle = true;
    Telemetry telemetry;

    public V4B_Arm(HardwareMap map, Telemetry telemetry){
        rightArm = new Caching_Servo(map, "rightarm");
        leftArm = new Caching_Servo(map, "leftarm");
        leftGrab = new Caching_Servo(map,"leftgrab");
        rightGrab = new Caching_Servo(map,"rightgrab");

        this.telemetry = telemetry;

        rightArm.setZeros(.075, 1);
        leftArm.setZeros(.08, 1);
        //ZEROES 12/26
        //RIGHT: .075  IN     //  1  OUT
        //LEFT: 1 IN    // .08  OUT

        armIn();
        open();

        write();
    }

    public void grabPos(double val) {
        leftGrab.setPosition(1-val);
        rightGrab.setPosition(val);
    }

    public void armPos(double val){
        leftArm.setPosition(1-val);
        rightArm.setPosition(val);
    }

    public void grab() {
        leftGrab.setPosition(.67);
        rightGrab.setPosition(.38);
    }
    public void deposit() {
        leftGrab.setPosition(.5);
        rightGrab.setPosition(.5);
    }
    public void open(){
        leftGrab.setPosition(.54);
        rightGrab.setPosition(.48);
    }
    public void armIn() {
        armPos(0.01);
    }
    public void armOut() { armPos(.8); }

    public void operate(GamepadEx gamepad, GamepadEx gamepad2, Telemetry telemetry) {
        if(gamepad2.isPress(GamepadEx.Control.x)){
            duoToggle = !duoToggle;
        }
        if(gamepad.isPress(GamepadEx.Control.left_bumper)){
            grabberToggle += 1;
            time.reset();
        }
        if(gamepad.isPress(GamepadEx.Control.left_trigger)){
            grabberToggle -= 1;
            time.reset();
        }

        //DUOTOGGLE ---- TRUE: DEPOSIT BOTH        FALSE: DEPSOIT BOTTOM AND THEN TOP
        if(duoToggle){
            if(grabberToggle == -1){
                grabberToggle = 0;
            }
            if(grabberToggle == 0){
                armIn();
                open();
            }
            if(grabberToggle == 1){
                armIn();
                grab();
            }
            if(grabberToggle == 2){
                grab();
                armOut();
            }
            if(grabberToggle == 3){
                armOut();
                deposit();                  //both deposit
            }
            if(grabberToggle ==4){
                grab();
                if(time.time()>.2){
                    armIn();
                    if(time.time()>.5){
                        grabberToggle =0;
                    }
                }
                else{
                    armOut();
                }
            }
            if(grabberToggle ==5){
                grabberToggle =4;
            }
        }
        if(!duoToggle){
            if(grabberToggle == -1){
                grabberToggle = 0;
            }
            if(grabberToggle == 0){
                armIn();
                open();
            }
            if(grabberToggle == 1){
                armIn();
                grab();
            }
            if(grabberToggle == 2){
                grab();
                armOut();
            }
            if(grabberToggle == 3){
                armOut();
                rightGrab.setPosition(.5); //deposit
            }
            if(grabberToggle == 4){
                armOut();
                leftGrab.setPosition(.5);  //second deposit
            }
            if(grabberToggle ==5){
                grab();

                if(time.time()>.2){
                    armIn();
                    if(time.time()>.5){
                        grabberToggle =0;
                    }
                }
                else{
                    armOut();
                }
            }
            if(grabberToggle ==6){
                grabberToggle =5;
            }
        }


        if(duoToggle){
            telemetry.addData("duoToggle","2pixels");
        }
        if(!duoToggle){
            telemetry.addData("duoToggle","single");
        }
        telemetry.addData("grabberToggle",grabberToggle);
        telemetry.addData("Left servo pos", leftArm.servo.getPosition());
        telemetry.addData("Right servo pos", rightArm.servo.getPosition());
    }
    public void write(){
        rightArm.write();
        leftArm.write();
        leftGrab.write();
        rightGrab.write();
    }
}
