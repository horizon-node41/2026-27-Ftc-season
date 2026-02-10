package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Motion.drive;
import org.firstinspires.ftc.teamcode.Robot.Motion.autoSequence;
import org.firstinspires.ftc.teamcode.Robot.Motion.protoDrive;
@Autonomous(name = "Builder Pattern Auto")
public class autoTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Builder and hardware setup
        protoDrive myRobot = new protoDrive(hardwareMap);
        autoSequence sequence = new autoSequence(myRobot);

        waitForStart();

        // 3. Run the Chain
        sequence.driveTiles(1.5f)
                .rotateDegrees(90)
                .driveTiles(1);
    }
}