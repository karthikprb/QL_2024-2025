package org.firstinspires.ftc.teamcode.OpModes;




import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.PurePursuit.CurvePoint;
import org.firstinspires.ftc.teamcode.PurePursuit.RobotMovement;


import java.util.ArrayList;


@Autonomous(name="BlueFarSensorTest")
public class BlueFarSensorTest extends LinearOpMode {


    private enum State {
        DETECT,

        RIGHTSTRAFE,

        MID_OUT,

        LEFT_BACK,

        LEFT_FORWARD,

        LEFT_STRAFE,


        STRAFE_OUT,

        STACK,
        BARRIER_CROSS,

        DEPOSIT,

        BACKUP,

        FINISH,

        COME_BACK,

        RETURNSTACK,
        PARK,
        IDLE;
    }


    State mRobotState = State.DETECT;


    public Pose2d OFFSET = new Pose2d(0,0,0);


    //First Position (Center of Tile of the Three Pieces of Tape)
    public Pose2d STRAFE_BARRIER_RIGHT = new Pose2d(5, 50.5, Math.toRadians(90));

    public Pose2d STACK = new Pose2d(20.7, 51, Math.toRadians(90));

    public Pose2d STACK_2 = new Pose2d(21.3, 55, Math.toRadians(90));

    public Pose2d STACK_3 = new Pose2d(21.2, 49, Math.toRadians(90));


    ;

    public Pose2d CROSS_BARRIER = new Pose2d(-66, 55, Math.toRadians(90));

    public Pose2d CROSS_BARRIER_2 = new Pose2d(-59, 54, Math.toRadians(90));

    public Pose2d EXTRAPOSITION_1 = new Pose2d(-70, 50, Math.toRadians(90));


    public Pose2d EXTRABACKPOSITION_1 = new Pose2d(-65, 43, Math.toRadians(90));

    public Pose2d EXTRAPOSITION_2 = new Pose2d(-80, 35, Math.toRadians(90));



    public Pose2d RIGHT_PLACEMENT_FORWARD = new Pose2d(-5, 25, Math.toRadians(0));

    public Pose2d RIGHT_PLACEMENT_FORWARDUP = new Pose2d(0, 41, Math.toRadians(0));

    public Pose2d RIGHTPLACEMENTSTRAFE = new Pose2d(9.3, 34, Math.toRadians(0));


    public Pose2d MID_OUT = new Pose2d(-4.4, 52, Math.toRadians(0));

    public Pose2d MID_PLACEMENT = new Pose2d(0.1, 46, Math.toRadians(0));



    //Left Sequence

    public Pose2d LEFT_POS1 = new Pose2d(0.2, 24, Math.toRadians(0));
    public Pose2d LEFT_POS2TURN = new Pose2d(-0.1, 25.3, Math.toRadians(115));
    public Pose2d LEFT_POS3 = new Pose2d(-7, 26, Math.toRadians(115));

    public Pose2d LEFT_POS4 = new Pose2d(0.1, 26.1, Math.toRadians(90));




    //Middle Position Before Deposit
    public Pose2d DEPOSIT_MID = new Pose2d(-68, 27, Math.toRadians(90));


    public Pose2d LEFT_DEPOSIT = new Pose2d(-90.9, 13, Math.toRadians(90));


    //Actual Middle Deposit
    public Pose2d MID_DEPOSIT =  new Pose2d(-91, 18, Math.toRadians(90));


    public Pose2d EXTRA_MID =  new Pose2d(-91, 15, Math.toRadians(90));


    public Pose2d RIGHT_DEPOSIT =  new Pose2d(-90.4, 18, Math.toRadians(90));

    public Pose2d EXTRA_RIGHT =  new Pose2d(-90, 21, Math.toRadians(90));





    public Pose2d PARK_RIGHT =  new Pose2d(-82.8, 25.2, Math.toRadians(90));

    public Pose2d PARK_MID =  new Pose2d(-83.75, 14, Math.toRadians(90));

    public Pose2d PARK_LEFT =  new Pose2d(-83, 19, Math.toRadians(90));

    public Pose2d FINAL_PARK =  new Pose2d(-83, 29, Math.toRadians(90));







    double pixelCase;
    double accum = 0;
    boolean gtp = false;
    boolean PPIND = true;
    boolean isDown = true;
    int cycle = 0;
    double prevy = 0;
    double y=0;
    double bufferHeading = 0;

    double pos = 0;
    double pos2 = .35;

    boolean go = true;

    double returning = 0;
    double moveSpeed = 1.0;
    double turnSpeed = 1.0;

    boolean distanceL;
    boolean distanceR;

    double distanceFront = 0;

    double distanceSide = 0;

    int sensorDetect = 0;
    TouchSensor frontTray;
    TouchSensor backTray;

    DistanceSensor wallStackDetect;

    DistanceSensor sideWallDetect;



    ElapsedTime time;

    ElapsedTime secondTime;

    ElapsedTime goTime;
    NormalizedColorSensor colorSensor;
    Robot robot;


    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        time = new ElapsedTime();
        secondTime = new ElapsedTime();
        goTime = new ElapsedTime();
        robot.setStartPose(new Pose2d(0, 0, 0));
        robot.stopAndResetEncoders();


        boolean slidesKickout = false;


        robot.blueFarInitiailzeWebcam();
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Robot Pos", robot.getPos());
            frontTray = hardwareMap.get(TouchSensor.class, "trayfront");
            backTray = hardwareMap.get(TouchSensor.class, "trayback");
            robot.v4b.open();
            robot.v4b.armMid();
            robot.v4b.autoGrab();
            robot.intake.lift();
            try {
                pixelCase = robot.blueGetFarPixelCase();
            } catch(Exception e){
                pixelCase = BlueFarPathSensor.value;
            }
            pos = 0.45;
            telemetry.addData("Case", pixelCase);
            robot.update();
            robot.updatePos();
            telemetry.update();
        }


        waitForStart();

        try {
            robot.stopWebcam();
        } catch(Exception e){
            pixelCase = BlueFarPath.value;
        }



        time.startTime();
        secondTime.startTime();


        while (opModeIsActive()) {
            ArrayList<CurvePoint> points = new ArrayList<>();


            switch (mRobotState) {
                case DETECT:
                    points.add(new CurvePoint(new Pose2d(0, 0, 0),moveSpeed,turnSpeed,10));
                    if(pixelCase == 0) {
                        points.add(new CurvePoint(RIGHT_PLACEMENT_FORWARD, 0.75, 0.75, 10));
                        points.add(new CurvePoint(RIGHT_PLACEMENT_FORWARDUP, 0.6, 0.6, 10));
                        points.add(new CurvePoint(RIGHTPLACEMENTSTRAFE, 0.5, 0.5, 10));
                    } else if (pixelCase == 1){
                        points.add(new CurvePoint(MID_PLACEMENT, 0.4 ,0.4, 10));
                    } else if (pixelCase == 2){
                        points.add(new CurvePoint(LEFT_POS1, 0.6, 0.6, 10));
                    }

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 3.5){
                        if(pixelCase == 2){
                            if(time.time() > 0.5){
                                newState(State.STRAFE_OUT);
                            }
                        } else {
                            robot.v4b.autoRelease();
                            if (time.time() > 0.5) {
                                newState(State.STRAFE_OUT);
                            }
                        }
                    } else {
                        time.reset();
                    }

                    break;


                case STRAFE_OUT:
                    robot.v4b.open();
                    robot.intake.intake_dropper.setPosition(pos);

                    if(pixelCase == 0){
                        points.add(new CurvePoint(RIGHTPLACEMENTSTRAFE,0.75,0.75,10));
                        points.add(new CurvePoint(STRAFE_BARRIER_RIGHT,0.75,0.75,10));
                    } else if(pixelCase == 1){
                        points.add(new CurvePoint(MID_PLACEMENT,0.7,0.7,10));
                        points.add(new CurvePoint(MID_OUT,0.7,0.7,10));
                    } else if (pixelCase == 2){
                        points.add(new CurvePoint(LEFT_POS1,0.6,0.6,10));
                        points.add(new CurvePoint(LEFT_POS2TURN,0.6,0.6,10));
                    }

                    if(pixelCase == 2){
                        if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 3.5 && Math.abs(robot.getPos().getHeading() - Math.toRadians(90)) < Math.toRadians(3)){
                            newState(State.LEFT_BACK);
                        } else {
                            time.reset();
                        }
                    } else if(pixelCase == 1){
                        if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 2.5){
                            newState(State.MID_OUT);
                        } else {
                            time.reset();
                        }
                    } else {
                        if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 3.5 && Math.abs(robot.getPos().getHeading() - Math.toRadians(90)) < Math.toRadians(3)){
                            newState(State.STACK);
                        } else {
                            time.reset();
                        }
                    }


                    break;

                case LEFT_BACK:
                    points.add(new CurvePoint(LEFT_POS2TURN,0.6,0.6,10));
                    points.add(new CurvePoint(LEFT_POS3,0.6,0.6,10));

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 3.5){
                        robot.v4b.autoRelease();
                        if(time.time() > 0.5) {
                            newState(State.LEFT_FORWARD);
                        }
                    } else {
                        time.reset();
                    }
                    break;

                case LEFT_FORWARD:
                    points.add(new CurvePoint(LEFT_POS3,0.7,0.7,10));
                    points.add(new CurvePoint(LEFT_POS4,0.7,0.7,10));

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 3.5){
                        newState(State.LEFT_STRAFE);
                    } else {
                        time.reset();
                    }
                    break;

                case LEFT_STRAFE:
                    points.add(new CurvePoint(LEFT_POS4,0.7,0.7,10));
                    points.add(new CurvePoint(STRAFE_BARRIER_RIGHT,0.7,0.7,10));

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 3.5){
                        newState(State.STACK);
                    } else {
                        time.reset();
                    }
                    break;

                case MID_OUT:
                    points.add(new CurvePoint(MID_OUT,0.8,0.8,10));
                    points.add(new CurvePoint(STRAFE_BARRIER_RIGHT,0.8,0.8,10));

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 3.5 && Math.abs(robot.getPos().getHeading() - Math.toRadians(90)) < Math.toRadians(3)){
                        newState(State.STACK);
                    } else {
                        time.reset();
                    }
                    break;


                case STACK:
                    PPIND = true;
                    gtp = false;
                    distanceL = frontTray.isPressed();
                    distanceR = backTray.isPressed();
                    robot.v4b.open();
                    points.add(new CurvePoint(STRAFE_BARRIER_RIGHT,0.62, 0.62,10));

                    if(cycle == 1) {
                        if (robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 70) {
                            points.add(new CurvePoint(STACK, 0.4, 0.4, 10));
                        }
                    } else if (cycle == 2){
                        if (robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 70) {
                            points.add(new CurvePoint(STACK_2, 0.4, 0.4, 10));
                        }
                    } else {
                            points.add(new CurvePoint(STACK, 0.5, 0.5, 10));
                    }

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 2.5) {
                        goTime.startTime();
                        if (cycle == 1 || cycle == 2) {
                                    if (distanceL && distanceR) {
                                        robot.intake.intake_dropper.setPosition(0.3);
                                        newState(State.BARRIER_CROSS);
                                        pos = pos+.07;
                                        go = true;
                                    } else {
                                        if (cycle == 1) {
                                            if(go){
                                                pos+=.0007;
                                            }

                                            if(goTime.time()>.1){
                                                go = !go;
                                                goTime.reset();
                                            }
                                            robot.intake.intake_dropper.setPosition(pos);
                                        } else if (cycle == 2) {
                                            if(go){
                                                pos+=.0007;
                                            }

                                            if(goTime.time()>.1){
                                                go = !go;
                                                goTime.reset();
                                            }
                                            robot.intake.intake_dropper.setPosition(pos);
                                        }
                                    }
                            } else {

                                    if (distanceL && distanceR) {
                                        pos = pos2+.07;
                                        robot.intake.intake_dropper.setPosition(0.3);
                                    newState(State.BARRIER_CROSS);
                                    go = true;

                                }
                                 else {
                                     if(go){
                                         pos2+=.0007;
                                     }

                                        if(goTime.time()>.1){
                                            go = !go;
                                            goTime.reset();
                                        }

                                    robot.intake.intake_dropper.setPosition(pos2);

                                }
                            }
                        } else {
                            time.reset();
                            if (cycle == 0) {
                                robot.intake.intakeSet(1);
                            } else if (cycle == 1 || cycle == 2) {
                                robot.intake.intakeSet(1);
                            }
                        }
                    break;

                case BARRIER_CROSS:
                    PPIND = true;
                    gtp = false;
                    robot.intake.intake_dropper.setPosition(0.3);

                        points.add(new CurvePoint(STACK, 0.7, 0.7, 10));
                        points.add(new CurvePoint(CROSS_BARRIER, 0.7, 0.7, 10));

                    if (cycle == 0) {
                            if (pixelCase == 0) {
                                points.add(new CurvePoint(RIGHT_DEPOSIT, 0.63, 0.63, 10));
                            } else if (pixelCase == 1) {
                                points.add(new CurvePoint(MID_DEPOSIT, 0.63, 0.63, 10));
                            } else if (pixelCase == 2) {
                                points.add(new CurvePoint(LEFT_DEPOSIT, 0.63, 0.63, 10));
                            }
                        } else if (cycle == 1 || cycle == 2) {
                            if (pixelCase == 0) {
                                points.add(new CurvePoint(EXTRA_MID, 0.63, 0.63, 10));
                            } else if (pixelCase == 1) {
                                points.add(new CurvePoint(EXTRA_RIGHT, 0.63, 0.63, 10));
                            } else if (pixelCase == 2) {
                                points.add(new CurvePoint(EXTRA_RIGHT, 0.63, 0.63, 10));
                            }
                        }

                        if (robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 140) {
                            robot.intake.stop();
                            robot.v4b.flapOpen();
                        }

                        if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 190 && robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) > 140){
                            robot.intake.outtakeDeposit();

                        }

                    if (robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 170) {
                            robot.v4b.armIn();
                            if (secondTime.time() > 0.3 && secondTime.time() < 0.45) {
                                robot.v4b.grab();
                            } else if (secondTime.time() > 0.45 && secondTime.time() < 0.6) {
                                robot.v4b.open();
                            } else if (secondTime.time() > 0.6) {
                                robot.v4b.grab();
                            }
                        } else {
                            secondTime.reset();
                        }

                        if (robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 47) {
                            secondTime.reset();
                            robot.v4b.armOut();
                            if (cycle == 0) {
                                robot.slides.setPosition(160);
                            } else if (cycle == 1) {
                                robot.slides.setPosition(225);
                            } else if (cycle == 2) {
                                robot.slides.setPosition(225);
                            }
                            if (time.time() > 1.0) {
                                newState(State.DEPOSIT);
                            }
                        } else {
                            time.reset();
                            secondTime.reset();
                        }

                    break;

                case DEPOSIT:
                    PPIND = true;
                    gtp = false;
                    //points.add(new CurvePoint(DEPOSIT_MID,0.65,0.65,10));

                    if (cycle == 0) {
                        robot.slides.setPosition(160);
                    } else if (cycle == 1) {
                        robot.slides.setPosition(225);
                    } else if (cycle == 2) {
                        robot.slides.setPosition(225);
                    }

                    if(cycle == 0){
                    /*
                    if (pixelCase == 0) {
                        points.add(new CurvePoint(RIGHT_DEPOSIT, 0.65, 0.65, 10));
                    } else if (pixelCase == 1) {
                        points.add(new CurvePoint(MID_DEPOSIT, 0.3, 0.3, 10));
                    } else {
                        points.add(new CurvePoint(LEFT_DEPOSIT, 0.3, 0.3, 10));
                    }
                    */



                        if (pixelCase == 0) {
                            if (secondTime.time() <= 0.3) {
                                robot.v4b.armOut();
                            } else if (secondTime.time() > 0.45 && secondTime.time() < 1) {
                                robot.v4b.horitzontalRight();
                            } else if (secondTime.time() > 1 && secondTime.time() < 1.2) {
                                robot.v4b.openDepositTestHorizontal();
                            } else if (secondTime.time() > 1.2 && secondTime.time() < 1.4) {
                                robot.v4b.vertical();
                            } else if (secondTime.time() > 1.4) {
                                robot.v4b.armIn();
                                newState(State.FINISH);
                            }
                        } else if (pixelCase == 1) {
                            if (secondTime.time() <= 0.3) {
                                robot.v4b.armOut();
                            } else if (secondTime.time() > 0.45 && secondTime.time() < 1) {
                                robot.v4b.horizontalLeft();
                            } else if (secondTime.time() > 1 && secondTime.time() < 1.2) {
                                robot.v4b.openDepositTestHorizontal();
                            } else if (secondTime.time() > 1.2 && secondTime.time() < 1.4) {
                                robot.v4b.vertical();
                            } else if (secondTime.time() > 1.4) {
                                robot.v4b.armIn();
                                newState(State.FINISH);
                            }
                        } else if (pixelCase == 2) {
                            if (secondTime.time() <= 0.3) {
                                robot.v4b.armOut();
                            } else if (secondTime.time() > 0.45 && secondTime.time() < 1) {
                                robot.v4b.horizontalLeft();
                            } else if (secondTime.time() > 1 && secondTime.time() < 1.2) {
                                robot.v4b.openDepositTestHorizontal();
                            } else if (secondTime.time() > 1.2 && secondTime.time() < 1.4) {
                                robot.v4b.vertical();
                            } else if (secondTime.time() > 1.4) {
                                robot.v4b.armIn();
                                newState(State.FINISH);
                            }
                        }

                    } else {

                        if (secondTime.time() <= 0.3) {
                            robot.v4b.armOut();
                        } else if (secondTime.time() > 0.65 && secondTime.time() < 0.85) {
                            robot.v4b.openDepositTest();
                        } else if (secondTime.time() > 0.85) {
                            robot.v4b.armIn();
                            newState(State.FINISH);
                        }
                    }


                    break;

                case FINISH:
                    PPIND = true;
                    gtp = false;
                    if (robot.slides.touchSensor.isPressed()) {
                        robot.slides.setPower(0.0);
                    } else {
                        robot.slides.setPower(-0.7);
                    }

                    if(true){
                        cycle += 1;

                        if(cycle == 1 || cycle == 2){
                            newState(State.COME_BACK);
                        } else {
                            newState(State.IDLE);
                        }
                    }
                    break;

                case COME_BACK:
                    if (robot.slides.touchSensor.isPressed()) {
                        robot.slides.setPower(0.0);
                    } else {
                        robot.slides.setPower(-0.7);
                    }

                    PPIND = true;
                    gtp = false;
                    if(cycle == 1){
                        pos = 0.55;
                    } else if (cycle == 2){
                        pos = 0.6;
                    }
                    robot.intake.intake_dropper.setPosition(pos);
                    robot.v4b.armMid();
                    robot.v4b.open();

                    if(cycle == 1) {
                        if (pixelCase == 0) {
                            points.add(new CurvePoint(RIGHT_DEPOSIT, 0.7, 0.7, 10));
                        } else if (pixelCase == 1) {
                            points.add(new CurvePoint(MID_DEPOSIT, 0.8, 0.8, 10));
                        } else {
                            points.add(new CurvePoint(LEFT_DEPOSIT, 0.8, 0.8, 10));
                        }
                    } else if(cycle == 2){
                        if (pixelCase == 0) {
                            points.add(new CurvePoint(EXTRA_MID, 0.7, 0.7, 10));
                        } else if (pixelCase == 1) {
                            points.add(new CurvePoint(EXTRA_RIGHT, 0.8, 0.8, 10));
                        } else {
                            points.add(new CurvePoint(EXTRA_RIGHT, 0.8, 0.8, 10));
                        }
                    }


                    if(cycle == 1){
                        points.add(new CurvePoint(CROSS_BARRIER,0.7,0.85,10));
                    } else if (cycle == 2){
                        points.add(new CurvePoint(CROSS_BARRIER,0.7,0.85,10));
                    }

                    points.add(new CurvePoint(STRAFE_BARRIER_RIGHT,0.7,0.85,10));

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 50){
                        robot.intake.intakeSet(1.0);
                    }


                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 3){
                        newState(State.STACK);
                    }
                    else {
                        time.reset();
                    }
                    break;

                case RETURNSTACK:
                    PPIND = true;
                    gtp = false;
                    robot.intake.intakeSet(0.9);
                    robot.intake.intake_dropper.setPosition(pos);
                    if(cycle == 1){
                        points.add(new CurvePoint(CROSS_BARRIER,0.7,0.85,10));
                    } else if (cycle == 2){
                        points.add(new CurvePoint(CROSS_BARRIER_2,0.7,0.85,10));
                    }
                    points.add(new CurvePoint(STRAFE_BARRIER_RIGHT,0.7,0.85,10));


                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 3){
                        newState(State.STACK);
                    }
                    else {
                        time.reset();
                    }
                    break;


                case IDLE:
                    if (robot.slides.touchSensor.isPressed()) {
                        robot.slides.setPower(0.0);
                    } else {
                        robot.slides.setPower(-0.7);
                    }
                    robot.drive.setPower(0, 0, 0);
                    robot.slides.reset();
                    robot.drive.write();
                    break;
            }


            if (points.size() != 0) {
                if(PPIND){
                    RobotMovement.followCurve(points, robot, telemetry);
                }else if(gtp){
                    robot.GoTo(points.get(points.size() - 1).toPose(), new Pose2d(points.get(points.size() - 1).moveSpeed, points.get(points.size() - 1).moveSpeed, points.get(points.size() - 1).turnSpeed));
                }else{
                    RobotMovement.followCurveAngled(points, robot, telemetry);
                }
            } else if(mRobotState == State.PARK){
                robot.drive.write();
                robot.updatePos();
            }else {
                telemetry.addLine("HERE!!!!");
                robot.drive.setPower(0, 0, 0,0);
                robot.drive.write();
                robot.updatePos();
            }




            robot.slides.write();
            robot.v4b.write();
            robot.intake.write();
            robot.update();


            for(int i = 0; i < points.size(); i++){
                telemetry.addData("Point" + i, points.get(i).toString());
            }


            telemetry.addData("State", mRobotState);
            telemetry.addData("Position", robot.getPos());
            telemetry.addData("Heading Error", Math.abs(robot.getPos().getHeading() - Math.toRadians(270)));
            telemetry.addData("IsDown", isDown);
            //telemetry.addData("Touch Sensor", robot.slides.isDown());
            telemetry.addData("Buffer Heading", Math.toDegrees(bufferHeading));
            telemetry.addData("FollowMe", RobotMovement.getLastFollowMe());
            telemetry.update();
        }
    }


/*ti
  public void updateSlidesDown(){
      if(robot.slides.isDown()){
          robot.slides.reset();
          robot.slides.setPower(0.0);
      } else {
          robot.slides.setPower(-0.2501);
      }
  }


 */


    public void newState (State state){
        time.reset();
        mRobotState = state;
    }
}

@Config
class BlueFarPathSensor{
    //Set the set/start position of the servo in dashboard
    public static double value = 0;
}


