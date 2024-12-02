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

public class Airplane {
    Caching_Motor plane;
    //public Caching_Servo intake_dropper;
    /*
    1. 0.33
    2. 0.27
    3.
    4.
    5. 0.09

     */

    ElapsedTime time;
    Telemetry telemetry;

    boolean planeToggle = false;

    public Airplane(HardwareMap map, Telemetry telemetry){
        plane = new Caching_Motor(map, "plane");
        this.telemetry = telemetry;

        plane.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        time = new ElapsedTime();
        time.startTime();

    }

    public void operate(GamepadEx gamepad2Ex, Telemetry telemetry){

        if(gamepad2Ex.isPress(GamepadEx.Control.left_trigger)){
            planeToggle = !planeToggle;
        }

        if(planeToggle){
            plane.setPower(-1);
        } else {
            plane.setPower(0);
        }


        telemetry.addData("Plane Toggle", planeToggle);
    }





    public void write(){
        plane.write();

    }
}
