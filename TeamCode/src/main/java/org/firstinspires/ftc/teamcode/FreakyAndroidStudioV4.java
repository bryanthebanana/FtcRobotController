package org.firstinspires.ftc.teamcode;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.robotcore.external.JavaUtil;


@Disabled
@TeleOp
public class FreakyAndroidStudioV4 extends OpMode {
    private DcMotor backleft;
    private DcMotor frontleft;
    private DcMotor backright;
    private DcMotor frontright;
    private DcMotorEx Flywheel;
    private CRServo leftservo;
    private CRServo rightservo;
    float y;
    float x;
    float rotation;
    double TPS;

    double led_light = 0;
    boolean FlyWheelReady;

    double flywheel_rpm = 1300;

    double flywheel_lower_bound = flywheel_rpm - 25;

    double flywheel_upper_bound = flywheel_rpm + 25;

    private Servo light;


    @Override
    public void init() {
        backleft = hardwareMap.get(DcMotor.class, "back left");
        frontleft = hardwareMap.get(DcMotor.class, "front left");
        backright = hardwareMap.get(DcMotor.class, "back right");
        frontright = hardwareMap.get(DcMotor.class, "front right");
        Flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        leftservo = hardwareMap.get(CRServo.class, "left servo");
        rightservo = hardwareMap.get(CRServo.class, "right servo");
        light = hardwareMap.get(Servo.class, "light");


        backleft.setDirection(DcMotor.Direction.FORWARD);
        frontleft.setDirection(DcMotor.Direction.FORWARD);
        backright.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.REVERSE);
        Flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FlyWheelReady = false;
        leftservo.setPower(0);
        rightservo.setPower(0);
    }

    public void loop(){
        y = -gamepad1.left_stick_y;
        x = gamepad1.left_stick_x;
        rotation = -gamepad1.right_stick_x;
        TPS = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(rotation))), 1));
        //backright.setPower((y + x + rotation) / TPS);
        //frontright.setPower(((y - x) + rotation) / TPS);
        //backleft.setPower(((y - x) - rotation) / TPS);
        //frontleft.setPower(((y + x) - rotation) / TPS);
        try {
            FlyWheelProgramming();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    private void FlyWheelProgramming() throws InterruptedException {
        light.setPosition(led_light);
        telemetry.addData("RPM", Flywheel.getVelocity());
        telemetry.addData("Target Power", flywheel_rpm);
        // Temp code
        if (Flywheel.getVelocity() <= flywheel_upper_bound && Flywheel.getVelocity() >= flywheel_lower_bound){
            led_light = 0.5;
        }
        else{
            led_light = 0;
        }
        if (gamepad1.xWasPressed()) {
            new PIDF_Flywheel();
            if (!FlyWheelReady) {
                Flywheel.setVelocity(flywheel_rpm);
                FlyWheelReady = true;
            } else {
                if (Flywheel.getVelocity() <= flywheel_upper_bound && Flywheel.getVelocity() >= flywheel_lower_bound) {
                    leftservo.setPower(-1);
                    rightservo.setPower(1);
                    sleep(400);
                    rightservo.setPower(0);
                    leftservo.setPower(0);
                }
            }
        }
        if (gamepad1.yWasPressed()) {
            leftservo.setPower(0);
            rightservo.setPower(0);
            Flywheel.setPower(0);
            FlyWheelReady = false;
        }
        telemetry.update();

    }
}
