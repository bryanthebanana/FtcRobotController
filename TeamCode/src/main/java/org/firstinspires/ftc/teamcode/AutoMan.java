package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class AutoMan extends OpMode {
    double F = 11.00;
    double P = 122.00;

    double y; double x; double rotate;

    // Declare OpMode members.
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private DcMotorEx launcher;
    private CRServo leftFeeder;
    private CRServo rightFeeder;
    enum State {
        WAIT_FOR_A,
        WAIT_FOR_X,
        FINISHED

    }
    State state = State.WAIT_FOR_A;
    @Override
    public void init(){
        state = State.WAIT_FOR_A;
    }

    @Override
    public void loop(){
        telemetry.addData("Current State", state);
        switch (state){
            case WAIT_FOR_A:
        }
    }
}
