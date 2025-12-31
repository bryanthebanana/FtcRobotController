package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class PIDF_Flywheel extends OpMode {


    final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;
    final double LAUNCHER_TARGET_VELOCITY = 1220;
    final double LAUNCHER_MIN_VELOCITY = LAUNCHER_TARGET_VELOCITY-50;
    double curTargetVelocity = LAUNCHER_TARGET_VELOCITY;
    double lowVelocity = 200;
    double[] stepSizes = {10, 1.0, 0.1, 0.01, 0.001};
    ElapsedTime feederTimer = new ElapsedTime();
    int stepIndex;

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private LaunchState launchState;

    double F = 10.9;
    double P = 132.00;

    private DcMotorEx launcher;
    private CRServo leftFeeder;
    private CRServo rightFeeder;

    @Override
    public void init(){
        launchState = LaunchState.IDLE;

        leftFeeder = hardwareMap.get(CRServo.class, "left servo");
        rightFeeder = hardwareMap.get(CRServo.class, "right servo");
        launcher = hardwareMap.get(DcMotorEx.class, "Flywheel");

        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(BRAKE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
    }

    @Override
    public void loop(){
        launch(gamepad1.rightBumperWasPressed());

        if (gamepad1.yWasPressed()){
            if(curTargetVelocity == LAUNCHER_TARGET_VELOCITY) {
                curTargetVelocity = lowVelocity;
            } else { curTargetVelocity = LAUNCHER_TARGET_VELOCITY; }
        }

        if (gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()){
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        launcher.setVelocity(curTargetVelocity);

        double curVelocity = launcher.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Current Velocity", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addData("Tuning P", "%.4f (Dpad U/D)" , P);
        telemetry.addData("Tuning F", "%.4f (Dpad L/R)" , F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);





    }
    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                }
                break;
        }
    }

}
