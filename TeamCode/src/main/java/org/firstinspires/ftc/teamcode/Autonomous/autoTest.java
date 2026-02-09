package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot.Drive;

public class autoTest extends LinearOpMode {
    public void runOpMode(){
        Drive drive = new Drive(hardwareMap);
        waitForStart();
        
        if(opModeIsActive()){
            for (int i = 0; i < 4; i++) {
                drive.driveTiles(1);
                drive.rotateDegrees(90);
                drive.strafeTiles(1);
            }
        }
    }
}
