package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.PurePursuit.CurvePoint;

import java.util.ArrayList;


@Autonomous
public class Auto extends LinearOpMode {

    public enum Autocase{
        orig,left,right
    }
    Autocase robotCase = Autocase.orig;
    Robot robot;
    Pose2d orig = new Pose2d(0,0,0);
    Pose2d left = new Pose2d(-3,0,Math.toRadians(270));
    Pose2d right = new Pose2d(6,0,Math.toRadians(90));


    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        while(opModeIsActive()){
          switch (robotCase){
              case orig:
                  robot.diffy.intake();
                  robotCase = Autocase.left;
                  break;
              case left:

          }

        }
    }

}
