package org.firstinspires.ftc.teamcode.OpModes;

import android.text.method.Touch;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.Robot;

import java.util.List;

@Autonomous
(name = "DistanceSensorTest")
public class DistanceSensorTest extends LinearOpMode {
    //Robot robot;
    TouchSensor digitalSensorTwo;
    DistanceSensor frontSensor;

    TouchSensor digitalSensor;

    public static double kp = 0.005;//0.02;
    public static double ki = 0.0;
    public static double kd = 0.0004;//0.0008;

    PIDFController sensorL;

    PIDFController sensorR;
    double distanceR = 0;

    public Pose2d PRE_LOAD_CLEAR = new Pose2d(-1.5, 22, Math.toRadians(0));
    List<LynxModule> allHubs;


    @Override
    public void runOpMode() {
        //robot = new Robot(hardwareMap, telemetry);
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        //sensorL = new PIDFController(new PIDCoefficients(kp, ki, kd));
        digitalSensor = hardwareMap.get(TouchSensor.class, "trayfront");
        digitalSensorTwo = hardwareMap.get(TouchSensor.class, "trayback");
        frontSensor = hardwareMap.get(DistanceSensor.class, "front");
        //robot.setStartPose(new Pose2d(0, 0, 0));
/*
        while (!isStarted() && !isStopRequested()) {
     xq~az
            robot.update();
            robot.updatePos();
            telemetry.update();
        }

 */


        waitForStart();

        while (opModeIsActive()) {
            //distanceR = ((DistanceSensor)rightDistanceSensor).getDistance(DistanceUnit.CM);
            boolean POLOLUFront = digitalSensor.isPressed();
            boolean POLOLUBack = digitalSensorTwo.isPressed();
            double front = frontSensor.getDistance(DistanceUnit.INCH);


/*
            if (distanceL < 30) {
                telemetry.addData("YAYAYAYAYAYA", "YAYAYAYAYA");
            }

 */

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


            telemetry.addData("POLOLUFront", POLOLUFront);
            telemetry.addData("POLOLUBack", POLOLUBack);
            telemetry.addData("Front", front);
            telemetry.update();
            update();
        }

    }

    public void update() {
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }


    }
}
