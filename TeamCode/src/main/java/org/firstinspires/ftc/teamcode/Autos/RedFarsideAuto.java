package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeleOps.ArcadeDrive;
import org.firstinspires.ftc.teamcode.TeleOps.Launcher;

@Autonomous
public class RedFarsideAuto extends LinearOpMode{

    ArcadeDrive a1 = new ArcadeDrive();
    Launcher l1 = new Launcher();

    ElapsedTime launchTimer = new ElapsedTime();

    int LAUNCH_WAIT = 5;

    @Override
    public void runOpMode() {

        a1.init(hardwareMap);
        l1.init(hardwareMap);

        waitForStart();

        a1.straightEncoder(2530, 0.3);
        a1.rotateEncoder( 425,0.3);
        double angle = 315;
        while(opModeIsActive() && !a1.imuTurn(angle, 0.3)){
            idle();
            telemetry.addData("FL power", a1.getFLPower());
            telemetry.addData("Current Heading,", a1.getHeading());
            telemetry.addData("Target angle", angle);
            telemetry.update();
        }
        a1.straightEncoder(1800,0.4);
        launchTimer.reset();
        while (!l1.updateState(true, true)) {
            idle();
        }
        a1.straightEncoder(-510, 0.3);
        a1.strafeEncoder(-800, 0.3);

    }
}


