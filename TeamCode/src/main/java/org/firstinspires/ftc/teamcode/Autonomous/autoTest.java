package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.drive;
import org.firstinspires.ftc.teamcode.Robot.Motion.autoSequence;
@Autonomous(name = "Builder Pattern Auto")
public class autoTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        // 1. Setup Hardware
        drive myRobot = new drive(hardwareMap);

        // 2. Setup Builder
        autoSequence sequence = new autoSequence(myRobot);

        waitForStart();

        // 3. Run the Chain
        sequence.driveTiles(1.5f)
                .wait(500)
                .rotateDegrees(90)
                .driveTiles(1);
    }
}