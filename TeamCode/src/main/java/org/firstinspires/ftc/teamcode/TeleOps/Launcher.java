package org.firstinspires.ftc.teamcode.TeleOps;



import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Launcher {

    private DcMotorEx launcher;

    private CRServo leftFeeder, rightFeeder;

    private Servo led;

    final double FEED_TIME_SECONDS = 0.15; //The feeder servos run this long when a shot is requested.
    final double END_TIME = 1;
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 0.7;
    final double LAUNCHER_TARGET_VELOCITY = 1450 - 300 ;
    final double LAUNCHER_MIN_VELOCITY = LAUNCHER_TARGET_VELOCITY-50;
    final double F = 10.9;
    final double P = 132.00;
    ElapsedTime feederTimer = new ElapsedTime();
    final double betweenBallTime = 0.7;
    int numOfShots = 0;


    /*
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING
    }
    */

    private String launchState;

    public void init(HardwareMap hm){
        launcher = hm.get(DcMotorEx.class, "Flywheel");
        leftFeeder = hm.get(CRServo.class, "left servo");
        rightFeeder = hm.get(CRServo.class, "right servo");
        led = hm.get(Servo.class, "led");

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, 0, 0, F));

        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        launchState = "IDLE";
        stopFeeder();
        stopLauncher();
    }

    public void stopFeeder(){
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);
    }
    /* State machine to launch the ball
    IDLE -> SPIN_UP-> LAUNCH -> LAUNCHING -> LAUNCH or IDLE
     */
    public boolean updateState(boolean shotRequested, boolean macroRequested){
        switch (launchState) {
            case "IDLE":
                if (shotRequested){
                    numOfShots = 0;
                    launchState = "SPIN_UP";
                }
                break;
            case "SPIN_UP":
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = "LAUNCH";
                }
                break;
            case "LAUNCH":
                rightFeeder.setPower((FULL_SPEED));
                leftFeeder.setPower(FULL_SPEED);
                numOfShots++;
                feederTimer.reset();
                launchState = "LAUNCHING";
                break;
            case "LAUNCHING":
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                    if (macroRequested && numOfShots < 3){
                        if(feederTimer.seconds() > FEED_TIME_SECONDS + betweenBallTime){
                            launchState = "LAUNCH";
                        }
                    }
                    else {
                        if (feederTimer.seconds() > END_TIME){
                            launchState = "IDLE";
                            launcher.setVelocity(STOP_SPEED);
                            return true;
                        }
                    }
                }
                break;
        }
        return false;
    }
    public void startLauncher(){
        launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
    }

    public void stopLauncher(){
        launcher.setVelocity(STOP_SPEED);
    }

    public void rpmChecker() {
        if (launcher.getVelocity() >= LAUNCHER_MIN_VELOCITY) {
            led.setPosition(0.5);
        } else {
            led.setPosition(0.3);
        }
    }



}
