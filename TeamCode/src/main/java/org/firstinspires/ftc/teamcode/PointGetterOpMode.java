package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.AprilTagAutoAlignmentManager;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "PointGetterOpMode")
@Config

public class PointGetterOpMode extends OpMode {

    SampleMecanumDrive drive;
    public DcMotor frontLeft, frontRight, backLeft, backRight;
    public double yMovement;
    public double xMovement;
    public double rotation;
    public double drivePower;
    public boolean halfSpeedToggle = true;
    public boolean aLast = false;
    public boolean drivingReverse = false;
    public boolean yLast = false;
    public static double px, py, ph;

    AprilTagAutoAlignmentManager hPosCheck = new AprilTagAutoAlignmentManager();


    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(px, py, ph));

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft = hardwareMap.dcMotor.get("backLeft");
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRight = hardwareMap.dcMotor.get("backRight");
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {

        xMovement = gamepad1.left_stick_x;
        yMovement = gamepad1.left_stick_y;
        rotation = gamepad1.right_stick_x;
        drivePower = Math.max(Math.max(Math.abs(yMovement), Math.abs(xMovement)), Math.abs(rotation));

        if (!aLast && gamepad1.a) {
            halfSpeedToggle = !halfSpeedToggle;
        }
        if (halfSpeedToggle) {
            drivePower *= 0.5;
        }
        aLast = gamepad1.a;

        if (!yLast && gamepad1.y) {
            drivingReverse = !drivingReverse;
        }
        if (drivingReverse) {
            xMovement *= -1;
            yMovement *= -1;
        }
        yLast = gamepad1.y;

        telemetry.addData("halfSpeed", halfSpeedToggle);
        telemetry.addData("drivingReverse", drivingReverse);
        telemetry.addData("POINTX", drive.getPoseEstimate().getX());
        telemetry.addData("POINTY", drive.getPoseEstimate().getY());
        telemetry.addData("HEADING", drive.getPoseEstimate().getHeading());
        telemetry.update();

        drive.moveInTeleop(xMovement, yMovement, rotation, drivePower);

    }

}