package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArcadeDrive {

    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;



    private final double SPIN_DAMPING = 1.2;

    public void init(HardwareMap hm) {


        FL = hm.get(DcMotor.class, "front left");
        FR = hm.get(DcMotor.class, "front right");
        BL = hm.get(DcMotor.class, "back left");
        BR = hm.get(DcMotor.class, "back right");

        FL.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
