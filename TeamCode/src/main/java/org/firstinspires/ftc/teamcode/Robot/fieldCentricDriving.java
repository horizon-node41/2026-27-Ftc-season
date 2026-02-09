package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class fieldCentricDriving {
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private IMU imu;

    public void init(HardwareMap hwMap) {
        leftFront  = hwMap.get(DcMotor.class, "leftFront");
        leftBack   = hwMap.get(DcMotor.class, "leftBack");
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        rightBack  = hwMap.get(DcMotor.class, "rightBack");

        // Set directions (Adjust based on your build)
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        imu = hwMap.get(IMU.class, "imu");
        imu.resetYaw();
    }

    /** * Simple function for field-centric driving.
     * You just pass in stick values; the class handles the math.
     */
    public void driveFieldCentric(double x, double y, double rx) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement vector by the robot heading
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Denominator is the largest motor power
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);

        // Calculate power for each motor
        leftFront.setPower((rotY + rotX + rx) / denominator);
        leftBack.setPower((rotY - rotX + rx) / denominator);
        rightFront.setPower((rotY - rotX - rx) / denominator);
        rightBack.setPower((rotY + rotX - rx) / denominator);
    }

    public void stop() {
        driveFieldCentric(0, 0, 0);
    }
}