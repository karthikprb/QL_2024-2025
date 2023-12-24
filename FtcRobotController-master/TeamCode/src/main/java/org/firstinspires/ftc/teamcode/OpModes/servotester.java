package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class servotester extends LinearOpMode {

    public double pos1 = 1;

    Servo servo;


    @Override
    public void runOpMode() {
        servo = hardwareMap.servo.get("servo");

        waitForStart();
        while(opModeIsActive()) {
            servo.setPosition(pos1);
        }
    }
}
