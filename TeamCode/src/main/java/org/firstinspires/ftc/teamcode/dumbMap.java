package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.LevineLocalization.poses;

import java.util.ArrayList;

//h
@Config
public class dumbMap {
    //Define runtime
    public ElapsedTime runtime = new ElapsedTime();

    //Define opMode

    public OpMode opMode;
    public Pose2d startingPosition, startingPosition2,startingPosition3, boardBack;


    //Define all hardware
    public VoltageSensor batteryVoltageSensor;
    public DcMotor frontLeft, frontRight, backLeft, backRight, slide;
    public Servo clawHAngle, clawVAngle, slideLAngle, slideRAngle, clawL, clawR, plane;
    public WebcamName bonoboCam;
    public HuskyLens huskyLens;
   /* public Pose2d Bcone = new Pose2d(poses.BconePosex,poses.BconePosey, poses.BconePoseheading);
    public Pose2d Bconepoke = new Pose2d(poses.BconepokePosex,poses.BconepokePosey);
    public Pose2d Rcone = new Pose2d(poses.RconePosex,poses.RconePosey, poses.RconePoseheading);
    public Pose2d Rconepoke = new Pose2d(poses.RconepokePosex,poses.RconepokePosey, poses.RconepokePoseheading);
    public Pose2d startPose = new Pose2d(poses.xPosStartingPos,poses.yPosStartingPos, poses.headingStartingPos);

    */
    public DistanceSensor distanceSensor;
    public BNO055IMU gyro;//Can we do it?

    public boolean halfSpeedToggle = true;
    public boolean aLast = false;


    public boolean drivingReverse = false;
    public boolean yLast = false;



    public double yMovement;
    public double xMovement;
    public double rotation;
    public double drivePower;
    public double slidePower;
    public static double pos;
    public Telemetry telemetry;
    public static double stackNum = 0;


    public dumbMap(OpMode opMode) {
        this.opMode = opMode;
    }

    public dumbMap(LinearOpMode opMode) {this.opMode = opMode;}


    public void init() {

        pos = 0.8;

        frontLeft = this.opMode.hardwareMap.dcMotor.get("frontLeft");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight = this.opMode.hardwareMap.dcMotor.get("frontRight");
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft = this.opMode.hardwareMap.dcMotor.get("backLeft");
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRight = this.opMode.hardwareMap.dcMotor.get("backRight");
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        huskyLens = this.opMode.hardwareMap.get(HuskyLens.class, "huskylens");
        distanceSensor = this.opMode.hardwareMap.get(DistanceSensor.class, "distanceSensor");


        VoltageSensor batteryVoltageSensor = this.opMode.hardwareMap.voltageSensor.iterator().next();

        //telemetry.addData("Battery Voltage: ", batteryVoltageSensor.getVoltage());
        //telemetry.update();
    }

    public double averageLastContents(ArrayList<Double> arr, int LOOKBACK){
        int len = arr.size();
        int count = Math.min(len, LOOKBACK);
        double sum = 0;
        for(int i = len - count; i < len; i++){
            sum += arr.get(i);
        }
        return sum/count;
    }


    /*
    public void initForApril(){
        WebcamName bonoboCam = hardwareMap.get(WebcamName.class, "Webcam 1");
    }

     */



}
