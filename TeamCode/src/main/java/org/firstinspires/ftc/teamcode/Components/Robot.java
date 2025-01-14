package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Odometry.S4T_Encoder;
import org.firstinspires.ftc.teamcode.Odometry.S4T_Localizer;

import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;

public class   Robot {

    public Mecanum_Drive drive;

    public boolean robotCentric = false;

    public boolean driveToggle = true;


    public Diffy diffy;

    public Slides slides;


    Linkage linkage;

    OpenCvCamera webcam;
    OpenCvPipeline detector;
    private HardwareMap hardwareMap;

    private Telemetry telemetry;


    public static Pose2d startPos = new Pose2d(0, 0, 0);

    List<LynxModule> allHubs;

    public Robot(HardwareMap map, Telemetry telemetry){
        this.hardwareMap = map;
        this.telemetry = telemetry;
        startPos = new Pose2d(0, 0, 0);


        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

     //   encoderLY = new S4T_Encoder(map, "bleft");
       // encoderLX = new S4T_Encoder(map, "fleft");
       // encoderRY = new S4T_Encoder(map, "bright");
       // encoderRX = new S4T_Encoder(map, "fright");

        diffy = new Diffy(map);
        slides = new Slides(map,telemetry);
        drive = new Mecanum_Drive(map, telemetry);
        linkage = new Linkage(map,telemetry);


        telemetry.update();
    }

    public void operate(GamepadEx gamepad1ex, GamepadEx gamepad2ex) {
        telemetry.addLine("MAKE SURE TO HIT RIGHT TRIGGER");

        drive.drive(gamepad1ex,1,1);
        drive.write();

        diffy.operate(gamepad1ex,gamepad2ex);
        diffy.write();

        linkage.operate(gamepad1ex,gamepad2ex,telemetry);
        linkage.write();

        slides.operate(gamepad2ex);
        slides.write();

        gamepad1ex.loop();
        gamepad2ex.loop();


        if(gamepad1ex.isPress(GamepadEx.Control.start)){
            telemetry.addLine("Resetting...");
        }else{
            update();
        }

    }




    public void update(){
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }


    }







}


