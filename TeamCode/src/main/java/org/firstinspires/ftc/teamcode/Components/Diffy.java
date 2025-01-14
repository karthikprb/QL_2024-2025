package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

public class Diffy {


    Caching_Servo lDiff;
    Caching_Servo rDiff;

    Caching_Servo inGrabber;

    Caching_Servo grabber;
    Caching_Servo inRightArm;
    Caching_Servo inLeftArm;
    Caching_Servo rightArm;
    Caching_Servo leftArm;

    ElapsedTime time;

    public static double in = 1;
    public static double out = .13;
    public static double transfer = 0.98; // left = .98, right = .02
    public static double grabPos = .35; // outtake grab pos
    public static double openPos = 0; // outtake open pos
    public static double intake = .13; // left = .55, right = .45

    public static double hover = 0.15;
    public static double inGrab = 0.12;
    public static double inOpen = 0.35;
    double rot = 0;
    double diffToggle = 0;
    public static double down = 0.05;
    public static double up = 0.65;
    public static double cw55 = 0.48; // left = .48, right = .38
    public static double ccw55 = 0.68; // left = .68, right = .58

    Telemetry telemetry;

    public Diffy(HardwareMap map){
        rDiff = new Caching_Servo(map, "rDiff");
        lDiff = new Caching_Servo(map, "lDiff");
        inGrabber = new Caching_Servo(map,"inGrabber");
        rightArm = new Caching_Servo(map, "rightarm");
        inLeftArm = new Caching_Servo(map, "inLeft");
        inRightArm = new Caching_Servo(map, "inRight");
        leftArm = new Caching_Servo(map, "leftarm");
        grabber = new Caching_Servo(map,"grabber");
        write();
        time = new ElapsedTime();

    }

    public void grab() {
        grabber.setPosition(grabPos);

    }

    public void open(){
        grabber.setPosition(openPos);

    }
    public void down(){
        inRightArm.setPosition(down);
        inLeftArm.setPosition(down);
    }
    public void up(){
        inRightArm.setPosition(up);
        inLeftArm.setPosition(up);
    }


    public void armIn() {
        leftArm.setPosition(in);
        rightArm.setPosition(in);
    }
    public void armOut() {
        leftArm.setPosition(out);
        rightArm.setPosition(out);
    }
    public void inGrab(){
        inGrabber.setPosition(inGrab);
    }

    public void inOpen(){

        inGrabber.setPosition(inOpen);
    }

    public void intake() {
        rDiff.setPosition(intake - .1);
        lDiff.setPosition(intake);
    }
    public void outtake() {
        rDiff.setPosition(transfer - .96);
        lDiff.setPosition(transfer);
    }

    public void hover(){
        inLeftArm.setPosition(hover);
        inRightArm.setPosition(hover);
    }

    public void rot(GamepadEx gamepadEx2){
        if(inRightArm.getPosition() == hover && inLeftArm.getPosition() == hover) {
            if (gamepadEx2.gamepad.left_bumper) {
                lDiff.setPosition(ccw55);
                rDiff.setPosition(ccw55 - .1);
            }else if (gamepadEx2.gamepad.right_bumper) {
                lDiff.setPosition(cw55);
                rDiff.setPosition(cw55 - .1);
            }else{
                intake();
            }
        }
    }

    public void operate(GamepadEx gamepadEx1, GamepadEx gamepadEx2) {
        if(gamepadEx1.isPress(GamepadEx.Control.right_bumper)){
            diffToggle++;
            time.reset();
        }
        if(diffToggle == 0){
            armIn();
            open();
            hover();
            rot(gamepadEx2);
            inOpen();
        }else if(diffToggle == 1){
            armIn();
            open();
            down();
            rot(gamepadEx2);
            if(time.time() > .3){
                inGrab();
            }
        }else if(diffToggle == 2){
            armIn();
            up();
            outtake();

            if(time.time()<0.5) {
                open();
                inGrab();
            }else if(time.time()>0.5 && time.time()<0.8){
                grab();
                inGrab();
            }else if(time.time()>0.8){
                inOpen();
                grab();
            }

        }else if(diffToggle == 3){
            armOut();
            up();
            outtake();
            grab();
        }else if(diffToggle == 4){
            armOut();
            up();
            outtake();
            open();
        }else if(diffToggle > 4){
            diffToggle = 0;
        }



    }

    public void write(){
        rDiff.write();
        lDiff.write();
        inGrabber.write();
        rightArm.write();
        inRightArm.write();
        leftArm.write();
        inLeftArm.write();
        grabber.write();
    }
}
