package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ArcadeDrive {

    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;
    private IMU imu;

    private DistanceSensor distanceSensor;

    double startHeading;

    double straightLineTolerance = 5;





    private final double SPIN_DAMPING = 1.2;

    public void init(HardwareMap hm) {


        FL = hm.get(DcMotor.class, "front left");
        FR = hm.get(DcMotor.class, "front right");
        BL = hm.get(DcMotor.class, "back left");
        BR = hm.get(DcMotor.class, "back right");
        imu = hm.get(IMU.class, "imu");
        distanceSensor = hm.get(DistanceSensor.class, "distance");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        );

        FL.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu.initialize(new IMU.Parameters(RevOrientation));
        startHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void drive(double y, double x, double rotate) {

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rotate), 1);

        double leftFrontPower = (y + x + rotate) / denominator;
        double rightFrontPower = (y - x - rotate) / denominator;
        double leftBackPower = (y - x + rotate) / denominator;
        double rightBackPower = (y + x - rotate) / denominator;

        FL.setPower(leftFrontPower / SPIN_DAMPING);
        FR.setPower(rightFrontPower / SPIN_DAMPING);
        BL.setPower(leftBackPower / SPIN_DAMPING);
        BR.setPower(rightBackPower / SPIN_DAMPING);
    }

    public void straightEncoder(int ticks, double power) {


        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setTargetPosition(ticks);
        FR.setTargetPosition(ticks);
        BL.setTargetPosition(ticks);
        BR.setTargetPosition(ticks);

        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FL.setPower(power);
        FR.setPower(power);
        BL.setPower(power);
        BR.setPower(power);

        while ((FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy())) {

        }

        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);




    }

    public void strafeEncoder(int ticks, double power) {


        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setTargetPosition(ticks);
        FR.setTargetPosition(-ticks);
        BL.setTargetPosition(-ticks);
        BR.setTargetPosition(ticks);

        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FL.setPower(power);
        FR.setPower(power);
        BL.setPower(power);
        BR.setPower(power);

        while ((FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy())) {

        }

        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    public void rotateEncoder(int ticks, double power) {


        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setTargetPosition(ticks);
        FR.setTargetPosition(-ticks);
        BL.setTargetPosition(ticks);
        BR.setTargetPosition(-ticks);

        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FL.setPower(power);
        FR.setPower(power);
        BL.setPower(power);
        BR.setPower(power);

        while ((FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy())) {

        }

        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    public boolean imuTurn(double angle, double power){
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        double kp = 0.01 - 0.006;
        double error;
        double minPower = 0.2;



        error = angle - getHeading();

        if (error > 180) error -= 360;
        if (error < -180) error += 360;

        if (Math.abs(error) < 2.5){
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            stopMotors();
            return true;
        }

        double turnPower = kp * error;

        if (turnPower > power) turnPower = power;
        if (turnPower < -power) turnPower = -power;


        if (Math.abs(turnPower) < minPower) {
            turnPower = Math.signum(turnPower) * minPower;
        }


        FL.setPower(-turnPower);
        BL.setPower(-turnPower);
        FR.setPower(turnPower);
        BR.setPower(turnPower);
        return false;

    }

    public boolean straightLineSensor(double targetDistance, double maxPower){
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double kp = 0.016;
        double minPower = 0.2;
        double tolerance = straightLineTolerance;

        double currentDistance = getDistance();

        double error = targetDistance - currentDistance;

        double power = -kp * error;
        if (Math.abs(error) < tolerance){
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            stopMotors();
            return true;
        }

        if (power > maxPower) power = maxPower;
        if (power < -maxPower) power = -maxPower;

        if (Math.abs(power) < minPower && Math.abs(error) > tolerance){
            power = Math.signum(power) * minPower;
        }

        FL.setPower(power);
        BL.setPower(power);
        FR.setPower(power);
        BR.setPower(power);

        //EXPERIMENTAL:Angle correction
        //imuTurn(getHeading(), 0.3);

        return false;

    }

    public double getFLPower(){
        return FL.getPower();
    }

    public double getDistance(){
        return distanceSensor.getDistance(DistanceUnit.CM);
    }



    public double getHeading(){
        double relativeHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - startHeading;
        return (relativeHeading + 360) % 360;
    }

    public void stopMotors(){
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    /* public int FLReturnEncoders(){
        return FL.getCurrentPosition();
    }
    public double FRReturnEncoders(){
        return FR.getCurrentPosition();
    }
    public double BLReturnEncoders(){
        return BL.getCurrentPosition();
    }
    public double BRReturnEncoders(){

        return BR.getCurrentPosition();
    }*/
}
