package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeleOps.ArcadeDrive;
import org.firstinspires.ftc.teamcode.TeleOps.Launcher;



@Autonomous
public class BlueNearsideAuto extends LinearOpMode {

    ArcadeDrive a1 = new ArcadeDrive();
    Launcher l1 = new Launcher();

    ElapsedTime launchTimer = new ElapsedTime();
    int brakePointMiliseconds = 5000;

    int LAUNCH_WAIT = 5;

    @Override
    public void runOpMode() throws InterruptedException{

        a1.init(hardwareMap);
        l1.init(hardwareMap);


        waitForStart();

        Thread.sleep(brakePointMiliseconds);

        launchTimer.reset();
        while (!l1.updateState(true, true)) {

        }

        l1.stopLauncher();
        a1.straightEncoder(-510, 0.3);
        a1.strafeEncoder(800, 0.3);


    }
}

