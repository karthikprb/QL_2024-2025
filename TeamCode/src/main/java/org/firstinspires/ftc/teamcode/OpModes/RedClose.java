package org.firstinspires.ftc.teamcode.OpModes;




import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.PurePursuit.CurvePoint;
import org.firstinspires.ftc.teamcode.PurePursuit.RobotMovement;


import java.util.ArrayList;


@Autonomous(name="RedClose")
public class RedClose extends LinearOpMode {


    private enum State {
        DETECT,
        PLACEMENT_LEFT,
        PLACEMENT_LEFT_TURN,
        PLACEMENT_MID,
        PLACEMENT_MID_BACK,
        PLACEMENT_MID_TURN,
        PLACEMENT_RIGHT,
        PLACEMENT_RIGHT_TURN,
        DEPOSIT_MID,
        DEPOSIT,

        FINISH,
        PARK,
        IDLE;
    }


    State mRobotState = State.DETECT;


    public Pose2d OFFSET = new Pose2d(0,0,0);


    //First Position (Center of Tile of the Three Pieces of Tape)
    public Pose2d PLACEMENT_CENTER = new Pose2d(-0.01, 26, Math.toRadians(0));


    //Left Tape Position
    public Pose2d LEFT_PLACEMENT_TURN = new Pose2d(-0.01, 13, Math.toRadians(0));
    public Pose2d RIGHT_PLACEMENT = new Pose2d(15, 31, Math.toRadians(270));


    //Mid Tape Position
    public Pose2d MID_PLACEMENT = new Pose2d(-0.01, 32, Math.toRadians(0));
    public Pose2d MID_PLACEMENT_BACK = new Pose2d(-0.01, 24, Math.toRadians(0));

    public Pose2d MID_PLACEMENT_TURN = new Pose2d(-0.01, 24, Math.toRadians(270));


    public Pose2d LEFT_PLACEMENT = new Pose2d(-6, 27.5, Math.toRadians(270));


    //Middle Position Before Deposit
    public Pose2d DEPOSIT_MID = new Pose2d(27, 24, Math.toRadians(270));


    public Pose2d RIGHT_DEPOSIT = new Pose2d(33, 18, Math.toRadians(270));


    //Actual Middle Deposit
    public Pose2d MID_DEPOSIT =  new Pose2d(33, 25, Math.toRadians(270));


    public Pose2d LEFT_DEPOSIT =  new Pose2d(33.5, 34, Math.toRadians(270));


    public Pose2d PARK =  new Pose2d(31, 3, Math.toRadians(270));






    double pixelCase;
    boolean gtp = false;
    boolean PPIND = true;
    boolean isDown = true;
    int cycle = 0;
    double bufferHeading = 0;
    double moveSpeed = 1.0;
    double turnSpeed = 1.0;




    ElapsedTime time;
    NormalizedColorSensor colorSensor;
    Robot robot;


    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        time = new ElapsedTime();
        robot.setStartPose(new Pose2d(0, 0, 0));
        robot.stopAndResetEncoders();


        boolean slidesKickout = false;


        robot.redCloseInitializeWebcam();
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Robot Pos", robot.getPos());
            robot.intake.lift();
            robot.v4b.grab();
            robot.v4b.armIn();
            pixelCase = robot.redGetClosePixelCase(); //RedClosePath.value;
            robot.update();
            robot.updatePos();
            telemetry.addData("Case", pixelCase);
            telemetry.update();
        }


        waitForStart();


        robot.stopWebcam();


        time.startTime();


        while (opModeIsActive()) {
            ArrayList<CurvePoint> points = new ArrayList<>();


            switch (mRobotState) {
                case DETECT:
                    points.add(new CurvePoint(new Pose2d(0, 0, 0),moveSpeed,turnSpeed,10));
                    points.add(new CurvePoint(PLACEMENT_CENTER,0.4,0.4,10));
                    if(pixelCase == 1){
                        if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 2){
                            newState(State.PLACEMENT_LEFT);
                        }
                    } else if(pixelCase == 2){
                        if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 2){
                            newState(State.PLACEMENT_MID);
                        }
                    } else {
                        if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 2){
                            newState(State.PLACEMENT_RIGHT_TURN);
                        }
                    }
                    break;

                case PLACEMENT_LEFT_TURN:
                    points.add(new CurvePoint(PLACEMENT_CENTER,0.25,0.25,10));
                    points.add(new CurvePoint(MID_PLACEMENT_TURN,0.25,0.25,10));


                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 2.0 && Math.abs(robot.getPos().getHeading() + Math.toRadians(90)) > Math.toRadians(3)){
                        newState(State.DEPOSIT_MID);
                    } else {
                        time.reset();
                    }


                    break;

                case PLACEMENT_LEFT:
                    points.add(new CurvePoint(MID_PLACEMENT_TURN,0.25,0.25,10));
                    points.add(new CurvePoint(LEFT_PLACEMENT,0.25,0.25,10));


                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 2.0 && Math.abs(robot.getPos().getHeading() + Math.toRadians(90)) > Math.toRadians(3)){
                        if(time.time() > 2.5){
                            robot.intake.stop();
                            newState(State.DEPOSIT_MID);
                        } else {
                            robot.intake.outtakeDeposit();
                        }
                    } else {
                        time.reset();
                    }
                    break;


                case PLACEMENT_MID:
                    points.add(new CurvePoint(PLACEMENT_CENTER,0.4,0.4,10));
                    points.add(new CurvePoint(MID_PLACEMENT,0.4,0.4,10));


                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 1.5){
                        if(time.time() > 2.5){
                            robot.intake.stop();
                            newState(State.PLACEMENT_MID_BACK);
                        } else {
                            robot.intake.outtakeDeposit();
                        }


                    } else {
                        time.reset();
                    }


                    break;


                case PLACEMENT_MID_BACK:
                    points.add(new CurvePoint(MID_PLACEMENT,0.4,0.4,10));
                    points.add(new CurvePoint(MID_PLACEMENT_BACK,0.4,0.4,10));


                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 2.0){
                        newState(State.PLACEMENT_MID_TURN);
                    } else {
                        time.reset();
                    }


                    break;

                case PLACEMENT_MID_TURN:
                    points.add(new CurvePoint(MID_PLACEMENT_BACK,0.4,0.4,10));
                    points.add(new CurvePoint(MID_PLACEMENT_TURN,0.4,0.4,10));


                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 2.0 && Math.abs(robot.getPos().getHeading() + Math.toRadians(90)) > Math.toRadians(3)){
                        newState(State.DEPOSIT_MID);
                    } else {
                        time.reset();
                    }


                    break;

                case PLACEMENT_RIGHT_TURN:
                    points.add(new CurvePoint(PLACEMENT_CENTER,0.4,0.4,10));
                    points.add(new CurvePoint(MID_PLACEMENT_TURN,0.4,0.4,10));


                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 2.0 && Math.abs(robot.getPos().getHeading() + Math.toRadians(90)) > Math.toRadians(3)){
                        newState(State.PLACEMENT_RIGHT);
                    } else {
                        time.reset();
                    }


                    break;


                case PLACEMENT_RIGHT:
                    points.add(new CurvePoint(MID_PLACEMENT_TURN,0.4,0.4,10));
                    points.add(new CurvePoint(RIGHT_PLACEMENT,0.4,0.4,10));


                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 5.0 && Math.abs(robot.getPos().getHeading() + Math.toRadians(90)) > Math.toRadians(3)){
                        if(time.time() > 4){
                            robot.intake.stop();
                            newState(State.DEPOSIT_MID);
                        } else {
                            robot.intake.outtakeDeposit();
                        }
                    } else {
                        time.reset();
                    }
                    break;


                case DEPOSIT_MID:
                    if(pixelCase == 1){
                        points.add(new CurvePoint(LEFT_PLACEMENT,0.4,0.4,10));
                        points.add(new CurvePoint(DEPOSIT_MID,0.4,0.4,10));
                    } else if(pixelCase == 2){
                        points.add(new CurvePoint(MID_PLACEMENT,0.4,0.4,10));
                        points.add(new CurvePoint(DEPOSIT_MID,0.4,0.4,10));
                    } else {
                        points.add(new CurvePoint(RIGHT_PLACEMENT,0.4,0.4,10));
                        points.add(new CurvePoint(DEPOSIT_MID,0.4,0.4,10));
                    }


                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 2.0 && Math.abs(robot.getPos().getHeading() + Math.toRadians(90)) > Math.toRadians(3)){
                        newState(State.DEPOSIT);
                    } else {
                        time.reset();
                    }


                    break;


                case DEPOSIT:
                    points.add(new CurvePoint(DEPOSIT_MID,0.4,0.4,10));


                    if(pixelCase ==  1){
                        points.add(new CurvePoint(LEFT_DEPOSIT,0.4,0.4,10));
                    } else if (pixelCase == 2){
                        points.add(new CurvePoint(MID_DEPOSIT,0.4,0.4,10));
                    } else {
                        points.add(new CurvePoint(RIGHT_DEPOSIT,0.4,0.4,10));
                    }


                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 3. && Math.abs(robot.getPos().getHeading() + Math.toRadians(90)) > Math.toRadians(3)){
                        robot.slides.setPosition(315);
                        if(time.time() <= 1.5){
                            robot.v4b.armOut();
                        } else if (time.time() > 1.5 && time.time() < 2){
                            robot.v4b.deposit();
                        } else if (time.time() > 2 && time.time() < 2.5){
                            robot.v4b.grab();
                        } else if (time.time() > 2.5 && time.time() < 3.5){
                            robot.v4b.armIn();
                        } else if(time.time() > 3.5){
                            newState(State.FINISH);
                        }


                    } else {
                        time.reset();
                    }


                    break;


                case FINISH:
                    robot.slides.setPower(-0.7);
                    robot.intake.drop();
                    if(pixelCase == 1){
                        points.add(new CurvePoint(LEFT_DEPOSIT,0.7,0.7,10));
                    } else if (pixelCase == 2){
                        points.add(new CurvePoint(MID_DEPOSIT,0.7,0.7,10));
                    } else {
                        points.add(new CurvePoint(RIGHT_DEPOSIT,0.7,0.7,10));
                    }

                    points.add(new CurvePoint(PARK,0.7,0.7,10));



                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 3. ){
                        newState(State.IDLE);
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
class RedClosePath{
    //Set the set/start position of the servo in dashboard
    public static double value = 0;
}
