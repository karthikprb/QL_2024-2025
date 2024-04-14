package org.firstinspires.ftc.teamcode.Components;

import android.app.admin.DevicePolicyManager;
import android.hardware.Sensor;
import android.text.method.Touch;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.C;
import org.checkerframework.checker.units.qual.Time;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

public class V4B_Arm {

    Caching_Servo rightArm;
    Caching_Servo leftArm;
    Caching_Servo leftGrab;
    Caching_Servo rightGrab;

    Caching_Servo holder;

    Caching_Servo wrist;

    Caching_Servo flap;

    double distanceL = 0;
    double distanceR = 0;
    TouchSensor frontTray;
    TouchSensor backTray;

    double trayConfirm = 1;

    double previousConfirm = 0;

    ElapsedTime time = new ElapsedTime();
    ElapsedTime secondTime = new ElapsedTime();
    public static double grabberToggle = 0;

    public static double wristCase = 0;

    boolean duoToggle = true;

    boolean manualWrist = false;
    Telemetry telemetry;

    public V4B_Arm(HardwareMap map, Telemetry telemetry){
        rightArm = new Caching_Servo(map, "rightarm");
        leftArm = new Caching_Servo(map, "leftarm");
        leftGrab = new Caching_Servo(map,"leftgrab");
        rightGrab = new Caching_Servo(map,"rightgrab");
        wrist = new Caching_Servo(map,"wrist");
        flap = new Caching_Servo(map,"flap");
        holder = new Caching_Servo(map, "holder");
        frontTray = map.get(TouchSensor.class, "trayfront");
        backTray = map.get(TouchSensor.class, "trayback");

        this.telemetry = telemetry;

        //rightArm.setZeros(.083, 1);
        //leftArm.setZeros(.08, 1);


        //ZEROES 12/26
        //RIGHT: .075  IN     //  1  OUT
        //LEFT: 1 IN    // .08  OUT

        armIn();
        autoGrab();
        wristCase = 0;
        vertical();
        open();

        write();
    }

    public void grabPos(double val) {
        leftGrab.setPosition(1-val);
        rightGrab.setPosition(val);
    }
/*
    public void armPos(double val){
        leftArm.setPosition(1-val);
        rightArm.setPosition(val);
    }


 */


    public void autoGrab(){
        holder.setPosition(0.06);
    }

    public void autoRelease(){
        holder.setPosition(0.4);
    }

    public void grab() {
        leftGrab.setPosition(0.26);
        rightGrab.setPosition(0.24);
    }

    public void deposit() {
        leftGrab.setPosition(0.46);
        rightGrab.setPosition(0.55);
    }
    public void open(){
        leftGrab.setPosition(0.41);
        rightGrab.setPosition(0.6);
    }


    public void openDepositTest(){
        leftGrab.setPosition(0.5);
        rightGrab.setPosition(0.53);
    }

    public void openDepositTestHorizontal(){
        leftGrab.setPosition(0.42);
        rightGrab.setPosition(0.55);
    }

    public void armMid() {
        leftArm.setPosition(0.21);
        rightArm.setPosition(0.74);
    }

    public void armIn() {
        leftArm.setPosition(0.15);
        rightArm.setPosition(0.80);
    }

    public void armUp() {
        leftArm.setPosition(0.21);
        rightArm.setPosition(0.74);
    }
    public void armOut() {
        leftArm.setPosition(0.75);
        rightArm.setPosition(0.2);
    }


    public void sixtyDegreesLeft(){
        wrist.setPosition(0.57);
    }

    public void sixtyDegreesRight(){
        wrist.setPosition(0.65);
    }

    public void horizontalLeft(){
        wrist.setPosition(0.32);
    }

    public void horitzontalRight(){
        wrist.setPosition(0.88);
    }


    public void vertical(){
        wrist.setPosition(0.6);
    }

    public void flapClose(){
        flap.setPosition(0.84); //FLAP CLOSE = 0.84
    }

    public void flapOpen(){
        flap.setPosition(0.7); //FLAP OPEN = 0.7
    }

    public void operate(GamepadEx gamepad, GamepadEx gamepad2, Telemetry telemetry) {
        boolean frontSensor = frontTray.isPressed();
        boolean backSensor = backTray.isPressed();
        if(gamepad.isPress(GamepadEx.Control.b)){
            autoRelease();
        }
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

        if(gamepad2.isPress(GamepadEx.Control.right_trigger)){
            grabberToggle = 1;
            trayConfirm = 1;
        }

        if(gamepad2.isPress(GamepadEx.Control.dpad_down)){
            if(wristCase > 0){
                wristCase -= 1;
            } else {
                wristCase = 0;
            }
        }

        if(gamepad2.isPress(GamepadEx.Control.dpad_up)){
            if(wristCase < 4){
                wristCase += 1;
            } else {
                wristCase = 4;
            }
        }


        //DUOTOGGLE ---- TRUE: DEPOSIT BOTH        FALSE: DEPSOIT BOTTOM AND THEN TOP
        if(duoToggle){
            if(grabberToggle == -1){
                grabberToggle = 0;
            }
            if(grabberToggle == 0) {
                vertical();
                armMid();
                flapClose();
                open();

                if (!gamepad2.isPress(GamepadEx.Control.right_trigger)) {
                    if (frontSensor && backSensor) {
                        gamepad.gamepad.rumble(100);
                    }
                }
            }

            if (grabberToggle == 1 && trayConfirm == 1) {
                    vertical();
                    armIn();
                    if (time.time() > 0.3 && time.time() < 0.5) {
                        grab();
                    } else if (time.time() > 0.5 && time.time() < 0.7) {
                        open();
                    } else if (time.time() > 0.7) {
                        grab();
                        previousConfirm = 1;
                    }
                    flapOpen();
            } else if(grabberToggle == 1 && trayConfirm == 0){
                grabberToggle = 0;
            }

            if(grabberToggle == 2 && previousConfirm == 1){
                flapOpen();
                vertical();
                grab();
                armOut();
                if(time.time() > 1){
                        if(wristCase == 0){
                            vertical();
                        } else if (wristCase == 1){
                            sixtyDegreesLeft();
                        } else if(wristCase == 2){
                            sixtyDegreesRight();
                        } else if(wristCase == 3) {
                            horizontalLeft();
                        } else {
                            horitzontalRight();
                        }
                    }
            } else if(grabberToggle == 2 && previousConfirm == 0){
                grabberToggle = 1;
            }
            if(grabberToggle == 3){
                manualWrist = false;
                previousConfirm = 0;
                if(wristCase == 3 || wristCase == 4){
                    openDepositTestHorizontal();
                } else {
                    openDepositTest();
                }
                flapClose();//both deposit
            }
            if(grabberToggle == 4){
                vertical();
                wristCase = 0;
                grab();
                if(time.time()>.1){
                    armIn();
                    if(time.time()>.5){
                        grabberToggle =0;
                    }
                }
                else{
                    armOut();
                }
                flapClose();
            }
            if(grabberToggle ==5){
                grabberToggle = 4;
            }
        }
        if(!duoToggle){
            if(grabberToggle == -1){
                grabberToggle = 0;
            }
            if(grabberToggle == 0){
                armMid();
                if(time.time() > 2){
                    armIn();
                }
                open();
            }
            if(grabberToggle == 1){
                armUp();
                if(time.time() > 0.15){
                    grab();
                }
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
        telemetry.addData("Wrist Case", wristCase);

    }
    public void write(){
        rightArm.write();
        leftArm.write();
        leftGrab.write();
        rightGrab.write();
        wrist.write();
        flap.write();
        holder.write();
    }
}
