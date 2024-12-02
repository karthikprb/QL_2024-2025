package org.firstinspires.ftc.teamcode.OpModes;




import android.text.method.Touch;

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


@Autonomous(name="BlueClose")
public class BlueClose extends LinearOpMode {


    private enum State {
        DETECT,
        RIGHT_TURN,

        RIGHT_FORWARD,

        MOVE_TO_DEPOSIT,

        DEPOSIT,

        CROSS_BARRIER,

        STACK,

        RETURN_PRECROSS,

        RETURN,



        PARK,
        IDLE;
    }


    State mRobotState = State.DETECT;


    public Pose2d OFFSET = new Pose2d(0,0,0);


    //First Position (Center of Tile of the Three Pieces of Tape
    public Pose2d ORIGIN = new Pose2d(0.01, 5, Math.toRadians(0));


    public Pose2d LEFTPLACEMENTFORWARD = new Pose2d(-10.3, 23, Math.toRadians(0));

    public Pose2d MID_PLACEMENT = new Pose2d(-1.1, 29.9, Math.toRadians(0));



    //Left Sequence

    public Pose2d RIGHT_POS1 = new Pose2d(-10, 25, Math.toRadians(0));
    public Pose2d RIGHT_POS2TURN = new Pose2d(-10, 25.5, Math.toRadians(240));
    public Pose2d RIGHT_POS3 = new Pose2d(9.25, 23.5, Math.toRadians(240));

    public Pose2d LEFT_DEPOSIT = new Pose2d(4.6, 46, Math.toRadians(0));
    public Pose2d MID_DEPOSIT = new Pose2d(4.6, 46, Math.toRadians(0));
    public Pose2d RIGHT_DEPOSIT = new Pose2d(4.6, 46, Math.toRadians(0));

    public Pose2d CROSS_BARRIER = new Pose2d(4.6, 46, Math.toRadians(0));
    public Pose2d PRE_STACK = new Pose2d(4.6, 46, Math.toRadians(0));
    public Pose2d STACK = new Pose2d(4.6, 46, Math.toRadians(0));











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

    boolean stackPosition = false;

    double returning = 0;
    double moveSpeed = 1.0;
    double turnSpeed = 1.0;

    boolean distanceL;
    boolean distanceR;

    TouchSensor frontTray;
    TouchSensor backTray;

    DistanceSensor wallStackDetect;

    DistanceSensor robotDetect;



    ElapsedTime time;

    ElapsedTime secondTime;

    NormalizedColorSensor colorSensor;
    Robot robot;


    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        time = new ElapsedTime();
        secondTime = new ElapsedTime();
        robot.setStartPose(new Pose2d(0, 0, 0));
        robot.stopAndResetEncoders();


        boolean slidesKickout = false;


        robot.blueFarInitiailzeWebcam();
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Robot Pos", robot.getPos());
            frontTray = hardwareMap.get(TouchSensor.class, "trayfront");
            backTray = hardwareMap.get(TouchSensor.class, "trayback");
            wallStackDetect = hardwareMap.get(DistanceSensor.class, "front");
            //robotDetect = hardwareMap.get(DistanceSensor.class, "side_robotDetect");
            robot.v4b.open();
            robot.v4b.armMid();
            robot.v4b.autoGrab();
            robot.intake.lift();
            try {
                pixelCase = robot.blueGetFarPixelCase();
            } catch(Exception e){
                pixelCase = BlueClosePath.value;
            }
            pos = 0.39;
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
                        points.add(new CurvePoint(LEFTPLACEMENTFORWARD, 0.6, 0.6, 10));
                    } else if (pixelCase == 1){
                        points.add(new CurvePoint(MID_PLACEMENT, 0.55, 0.55, 10));
                    } else if (pixelCase == 2){
                        points.add(new CurvePoint(RIGHT_POS1, 0.6, 0.6, 10));
                    }

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 3.5){
                        if(pixelCase == 2){
                            robot.intake.intakeSet(-0.8);
                            if(time.time() > 2.1){
                                newState(State.RIGHT_TURN);
                            }
                        } else {
                            robot.intake.intakeSet(-0.8);
                            if (time.time() > 2.1) {
                                newState(State.MOVE_TO_DEPOSIT);
                            }
                        }
                    } else {
                        time.reset();
                    }

                    break;

                case RIGHT_TURN:
                    points.add(new CurvePoint(RIGHT_POS1, 0.6, 0.6, 10));
                    points.add(new CurvePoint(RIGHT_POS2TURN, 0.6, 0.6, 10));

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 3.5 && Math.abs(robot.getPos().getHeading() + Math.toRadians(90)) > Math.toRadians(3)){
                        newState(State.RIGHT_FORWARD);
                    } else {
                        time.reset();
                    }
                    break;

                case RIGHT_FORWARD:
                    points.add(new CurvePoint(RIGHT_POS2TURN, 0.6, 0.6, 10));
                    points.add(new CurvePoint(RIGHT_POS3, 0.6, 0.6, 10));

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 3.5){
                            robot.intake.intakeSet(-0.8);
                            if(time.time() > 2.1){
                                newState(State.MOVE_TO_DEPOSIT);
                            }
                        }
                    break;

                case MOVE_TO_DEPOSIT:
                    if(pixelCase == 0) {
                        points.add(new CurvePoint(LEFTPLACEMENTFORWARD, 0.6, 0.6, 10));
                        points.add(new CurvePoint(LEFT_DEPOSIT, 0.6, 0.6, 10));
                    } else if (pixelCase == 1){
                        points.add(new CurvePoint(MID_PLACEMENT, 0.55, 0.55, 10));
                        points.add(new CurvePoint(MID_DEPOSIT, 0.6, 0.6, 10));
                    } else if (pixelCase == 2){
                        points.add(new CurvePoint(RIGHT_POS3, 0.6, 0.6, 10));
                        points.add(new CurvePoint(RIGHT_DEPOSIT, 0.6, 0.6, 10));
                    }

                    if (robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 60) {
                        robot.intake.stop();
                        robot.intake.drop();
                        robot.v4b.flapOpen();
                    }

                    if (robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 47) {
                        secondTime.reset();
                        robot.v4b.armOut();
                        if (cycle == 0) {
                            robot.slides.setPosition(350);
                        } else if (cycle == 1) {
                            robot.slides.setPosition(530);
                        } else if (cycle == 2) {
                            robot.slides.setPosition(700);
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
                    if (cycle == 0) {
                        robot.slides.setPosition(350);
                    } else if (cycle == 1) {
                        robot.slides.setPosition(530);
                    } else if (cycle == 2) {
                        robot.slides.setPosition(700);
                    }


                    if (secondTime.time() <= 0.3) {
                        robot.v4b.armOut();
                    } else if (secondTime.time() > 0.65 && secondTime.time() < 0.85) {
                        robot.v4b.openDepositTest();
                    } else if (secondTime.time() > 0.85) {
                        robot.v4b.armIn();
                        cycle += 1;
                        if(cycle == 3) {
                            newState(State.IDLE);
                        } else {
                            newState(State.CROSS_BARRIER);
                        }
                    }

                    break;

                case CROSS_BARRIER:
                    if (cycle == 0) {
                        if (pixelCase == 0) {
                            points.add(new CurvePoint(LEFT_DEPOSIT, 0.63, 0.63, 10));
                        } else if (pixelCase == 1) {
                            points.add(new CurvePoint(MID_DEPOSIT, 0.63, 0.63, 10));
                        } else if (pixelCase == 2) {
                            points.add(new CurvePoint(RIGHT_DEPOSIT, 0.63, 0.63, 10));
                        }
                    } else if (cycle == 1) {
                        if (pixelCase == 0) {
                            points.add(new CurvePoint(RIGHT_DEPOSIT, 0.63, 0.63, 10));
                        } else if (pixelCase == 1) {
                            points.add(new CurvePoint(RIGHT_DEPOSIT, 0.63, 0.63, 10));
                        } else if (pixelCase == 2) {
                            points.add(new CurvePoint(MID_DEPOSIT, 0.63, 0.63, 10));
                        }
                    }

                    points.add(new CurvePoint(CROSS_BARRIER, 0.63, 0.63, 10));

                    points.add(new CurvePoint(PRE_STACK, 0.63, 0.63, 10));


                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 3){
                        newState(State.STACK);
                    }
                    else {
                        time.reset();
                    }
                    break;

                case STACK:
                    distanceL = frontTray.isPressed();
                    distanceR = backTray.isPressed();
                    robot.v4b.open();
                    points.add(new CurvePoint(PRE_STACK,0.62, 0.62,10));

                    if(cycle == 1) {
                        if (robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 70) {
                            points.add(new CurvePoint(STACK, 0.4, 0.4, 10));
                        }
                    } else if (cycle == 2){
                        if (robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 95) {
                            points.add(new CurvePoint(STACK, 0.4, 0.4, 10));
                        }
                    } else {
                        points.add(new CurvePoint(STACK, 0.5, 0.5, 10));
                    }

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 2.5) {
                        if (cycle == 1 || cycle == 2) {
                            if (time.time() > 1.3) {
                                if (distanceL && distanceR) {
                                    newState(State.RETURN_PRECROSS);
                                } else {
                                    if (cycle == 1) {
                                        pos += 0.001;
                                        robot.intake.intake_dropper.setPosition(pos);
                                    } else if (cycle == 2) {
                                        pos += 0.002;
                                        robot.intake.intake_dropper.setPosition(pos);
                                    }
                                }
                            }
                        } else {
                            if (distanceL && distanceR) {
                                robot.intake.intakeSet(0.0);
                                if(time.time() < 1.3){
                                    robot.intake.intake_dropper.setPosition(0.5);
                                } else {
                                    newState(State.RETURN_PRECROSS);
                                }
                            } else {
                                time.reset();
                                pos += 0.0008;
                                robot.intake.intake_dropper.setPosition(pos);
                            }
                        }
                    } else {
                        time.reset();
                        if (cycle == 0) {
                            robot.intake.intakeSet(0.9);
                        } else if (cycle == 1 || cycle == 2) {
                            robot.intake.intakeSet(0.9);
                        }
                    }
                    break;

                case RETURN_PRECROSS:

                    points.add(new CurvePoint(STACK, 0.4, 0.4, 10));
                    points.add(new CurvePoint(PRE_STACK, 0.4, 0.4, 10));

                    if (robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 60) {
                        robot.intake.stop();
                        robot.intake.drop();
                        robot.v4b.flapOpen();
                    }

                    if (robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 3) {
                        secondTime.reset();
                        newState(State.RETURN);
                    } else {
                        time.reset();
                        secondTime.reset();
                    }
                    break;

                case RETURN:
                    points.add(new CurvePoint(PRE_STACK, 0.4, 0.4, 10));
                    points.add(new CurvePoint(CROSS_BARRIER, 0.4, 0.4, 10));

                    if (cycle == 0) {
                        if (pixelCase == 0) {
                            points.add(new CurvePoint(MID_DEPOSIT, 0.63, 0.63, 10));
                        } else if (pixelCase == 1) {
                            points.add(new CurvePoint(LEFT_DEPOSIT, 0.63, 0.63, 10));
                        } else if (pixelCase == 2) {
                            points.add(new CurvePoint(LEFT_DEPOSIT, 0.63, 0.63, 10));
                        }
                    } else if (cycle == 1 || cycle == 2) {
                        if (pixelCase == 0) {
                            points.add(new CurvePoint(MID_DEPOSIT, 0.63, 0.63, 10));
                        } else if (pixelCase == 1) {
                            points.add(new CurvePoint(LEFT_DEPOSIT, 0.63, 0.63, 10));
                        } else if (pixelCase == 2) {
                            points.add(new CurvePoint(LEFT_DEPOSIT, 0.63, 0.63, 10));
                        }
                    }

                    if (robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 60) {
                        robot.intake.stop();
                        robot.intake.drop();
                        robot.v4b.flapOpen();
                    }

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 190){
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
                            robot.slides.setPosition(350);
                        } else if (cycle == 1) {
                            robot.slides.setPosition(530);
                        } else if (cycle == 2) {
                            robot.slides.setPosition(700);
                        }
                        if (time.time() > 1.0) {
                            newState(State.DEPOSIT);
                        }
                    } else {
                        time.reset();
                        secondTime.reset();
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
            telemetry.addData("StackCheck", stackPosition);

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
class BlueClosePath{
    //Set the set/start position of the servo in dashboard
    public static double value = 0;
}


