package org.firstinspires.org.firstinspires.ftc.Odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class S4T_Encoder {
    DcMotor encoder;
    public double distance;
    public double offset;
    boolean reverse = false;

    public S4T_Encoder(HardwareMap hardwareMap, String name){
        encoder = hardwareMap.get(DcMotorEx.class, name);
        //encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void reset(){
        offset = encoder.getCurrentPosition();
        distance = 0;
    }

    public void stopandreset(){
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update(){
        double val = encoder.getCurrentPosition();
        if(val != 0) {
            distance = val - offset;
        }
    }

    public double getDist(){
        if(reverse){
            return -(distance/1440) * ((58/25.4) * Math.PI);
        }else{
            return (distance/1440) * ((58/25.4) * Math.PI);
        }
    }
}