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
    int brakePointMiliseconds = 5000;
    boolean actuallyDoAuto = true; // change value if cracked out team
    boolean leftStrafe = true;//trafe left at end if true, strafe right at end if false
    int LAUNCH_WAIT = 5;

    int farSideDistanceToWall = 78;

    @Override
    public void runOpMode() throws InterruptedException{

        a1.init(hardwareMap);
        l1.init(hardwareMap);

        waitForStart();
        if(actuallyDoAuto){
          //  Thread.sleep(brakePointMiliseconds);

            // 1. Move straight using encoders
            a1.straightEncoder(2825, 0.6);
            // 2. Ensure the robot is at the correct distance using the distance sensor measuring off the side wall
            if (Math.abs(a1.getDistance() - farSideDistanceToWall) >= 8){
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
            a1.straightEncoder(1500,0.4);
            // 6. Launch all three balls
            launchTimer.reset();
            while (!l1.updateState(true, true)) {
                idle();
            }
            // 7. Move backwards away from the goal
            a1.straightEncoder(-510, 0.3);
            // 8. Strafe left into the zone

            if(leftStrafe){
                a1.strafeEncoder(-800, 0.3);
            } else {
                a1.strafeEncoder(800, 0.3);
            }
        }
        else {
            a1.straightEncoder(400, 0.3);
        }

    }
}


