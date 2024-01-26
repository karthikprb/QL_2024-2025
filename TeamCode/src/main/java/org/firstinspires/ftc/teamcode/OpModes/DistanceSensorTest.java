package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.Robot;

@Autonomous
        (name = "DistanceSensorTest")
public class DistanceSensorTest extends LinearOpMode {
    Robot robot;
    NormalizedColorSensor leftDistanceSensor;
    NormalizedColorSensor rightDistanceSensor;

    public static double kp = 0.005;//0.02;
    public static double ki = 0.0;
    public static double kd = 0.0004;//0.0008;

    PIDFController sensorL;

    PIDFController sensorR;
    double distanceL = 0;
    double distanceR = 0;
    public Pose2d PRE_LOAD_CLEAR = new Pose2d(-1.5, 22, Math.toRadians(0));


    @Override
    public void runOpMode(){
        robot = new Robot(hardwareMap, telemetry);
        sensorL = new PIDFController(new PIDCoefficients(kp, ki, kd));
        leftDistanceSensor = hardwareMap.get(NormalizedColorSensor.class, "LeftSensor");
        rightDistanceSensor = hardwareMap.get(NormalizedColorSensor.class, "RightSensor");
        robot.setStartPose(new Pose2d(0, 0, 0));

        while (!isStarted() && !isStopRequested()) {
            robot.update();
            robot.updatePos();
            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive()){
            distanceL = ((DistanceSensor)leftDistanceSensor).getDistance(DistanceUnit.INCH);
            distanceR = ((DistanceSensor)rightDistanceSensor).getDistance(DistanceUnit.INCH);


            /*
            if(distanceL > 1 && distanceR < 1){
                robot.GoToAlign(robot.getPos().getX(),robot.getPos().getY(),robot.getPos().getHeading()+1,1,1,1);
                telemetry.addData("Turn Right", "Turn Right");
            } else if (distanceL < 1 && distanceR > 1){

                telemetry.addData("Turn Left", "Turn Left");
            } else {
                telemetry.addData("Complete", "Complete");
            }

             */


            telemetry.addData("Distance Left", distanceL);
            telemetry.addData("Distance Right", distanceR);
            telemetry.update();
        }

    }




}
