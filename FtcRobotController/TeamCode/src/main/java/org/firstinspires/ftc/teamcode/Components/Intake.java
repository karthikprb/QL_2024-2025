package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

public class Intake {

   Caching_Servo leftDrop;
   Caching_Servo rightDrop;
   Caching_Servo leftRot;
   Caching_Servo rightRot;
   Caching_Servo grabber;

   double drop = 0;
   double lift = 0.7;
   double grab = 0;

   double move = 0;
   double grabbing = 0;
   double release = 0;
   double leftPos = 0;
   double rightPos = 0;
    public Intake(HardwareMap map, Telemetry telemetry){
        leftDrop = new Caching_Servo(map,"leftDrop");
        rightDrop = new Caching_Servo(map,"rightDrop");
        leftRot = new Caching_Servo(map,"leftPick");
        rightRot = new Caching_Servo(map,"rightPick");
        grabber = new Caching_Servo(map,"grabber");
    }

    public void drop() {
    leftDrop.setPosition(drop);
    rightDrop.setPosition(drop);
    }
    public void lift(){
        leftDrop.setPosition(lift);
        rightDrop.setPosition(lift);
    }
    public void grab(){
        grabber.setPosition(grab);
    }
    public void release(){
        grabber.setPosition(release);
    }
    public void up(){

        leftRot.setPosition(leftPos);
        rightRot.setPosition(rightPos);
    }
    public void down(){
        leftPos = leftPos -.5;
        rightPos = rightPos -.5;
        leftRot.setPosition(leftPos);
        rightRot.setPosition(rightPos);
    }
    public void left(){
        leftPos = leftPos + .1;
        rightPos = rightPos - .1;
        leftRot.setPosition(leftPos);
        rightRot.setPosition(rightPos);
    }

    public void right(){
        leftPos = leftPos -.1;
        rightPos = rightPos + .1;
        leftRot.setPosition(leftPos);
        rightRot.setPosition(rightPos);
    }

    public void intake(GamepadEx gamepadEx, GamepadEx gamepad2Ex, Telemetry telemetry){

        if(gamepadEx.isPress(GamepadEx.Control.right_bumper)){
        move++;
          if(move == 1){
              lift();
          }else if(move == 2){
              drop();
          } else if (move == 3) {
              move = 0;
            }
        }if(gamepadEx.isPress(GamepadEx.Control.left_bumper)){
            grabbing++;
            if(grabbing == 1){
                release();
            }else if (grabbing == 2){
                grab();
            } else if (grabbing == 3) {
                grabbing = 0;
            }

        }

    }

    public void write(){
        leftRot.write();
        rightRot.write();
        leftDrop.write();
        rightDrop.write();
    }
}
