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

    int farSideDistanceToWall = 85;

    @Override
    public void runOpMode() {

        a1.init(hardwareMap);
        l1.init(hardwareMap);

        waitForStart();

        // 1. Move straight using encoders
        a1.straightEncoder(2530, 0.3);
        // 2. Ensure the robot is at the correct distance using the distance sensor measuring off the side wall
        if (Math.abs(a1.getDistance() - farSideDistanceToWall) > 5){
            while(opModeIsActive() && !a1.straightLineSensor(farSideDistanceToWall, 0.3)){
                idle();
            }
        }
        // 3. Ensure the robot is pointing straight after moving straight for a while
        while(opModeIsActive() && !a1.imuTurn(0, 0.3)){
            idle();
        }
        // 4. Turn to face the goal
        double angle = 315;
        while(opModeIsActive() && !a1.imuTurn(angle, 0.3)){
            idle();
        }
        // 5. Move straight until touching the goal
        //a1.straightEncoder(1800,0.4);
        while(opModeIsActive() && !a1.straightLineSensor(0,0.3)){
            idle();
        }
        // 6. Launch all three balls
        launchTimer.reset();
        while (!l1.updateState(true, true)) {
            idle();
        }
        // 7. Move backwards away from the goal
        a1.straightEncoder(-510, 0.3);
        // 8. Strafe left into the zone
        a1.strafeEncoder(-800, 0.3);

    }
}


