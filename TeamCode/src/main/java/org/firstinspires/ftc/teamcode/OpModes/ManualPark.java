package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Robot;
@Autonomous(name="ManualPark")
public class ManualPark extends LinearOpMode {
    Robot robot;
    ElapsedTime time;
    double pixels;
    @Override
    public void runOpMode(){
        robot = new Robot(hardwareMap,telemetry);
        time = new ElapsedTime();
        robot.intake.lock();


        robot.initializeWebcam();
        while (!isStarted() && !isStopRequested()) {
            robot.intake.lock();
            robot.update();
            pixels = robot.getPixelCase();
            telemetry.addData("Case", pixels);
            telemetry.update();
        }

        time.startTime();

        while(opModeIsActive()){
            robot.stopWebcam();
            robot.intake.unlock();
            if(pixels ==1){
                if(time.time()<5){
                    robot.drive.setPower(0,.2,330);
                }else{
                    robot.intake.outtakeDeposit();
                    robot.drive.setPower(0,0,0);
                }
            }else if(pixels ==2){
                if(time.time()<5){
                    robot.drive.setPower(0,.2,360);
                }else{
                    robot.intake.outtakeDeposit();
                    robot.drive.setPower(0,0,0);
                }
            }else{
                if(time.time()<5){
                    robot.drive.setPower(-1,.2,30);
                }else{
                    robot.intake.outtakeDeposit();
                    robot.drive.setPower(0,0,0);
                }
            }

            robot.drive.write();
        }


    }


}
