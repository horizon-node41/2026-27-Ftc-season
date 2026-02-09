package org.firstinspires.ftc.teamcode.Robot;
import static android.os.SystemClock.sleep;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.function.Consumer;
import java.util.function.Function;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class Drive {
    HardwareMap hMap;
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor rightFront;
    IMU imu;

    // The value that we multiply the encoder by to get one tile
    static final float WHEEL_MOTOR_ENCODER_SCALING = 1003.046f;

    public Drive(HardwareMap hardwareMap) {
        hMap = hardwareMap;
        leftFront = hMap.get(DcMotorEx.class, "leftFront");
        leftBack = hMap.get(DcMotorEx.class, "leftBack");
        rightBack = hMap.get(DcMotorEx.class, "rightBack");
        rightFront = hMap.get(DcMotorEx.class, "rightFront");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);
        forEachMotor(m -> m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER));
    }

    public void driveTiles(float Tiles) {
        forEachMotor(m -> m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        forEachMotor(m -> m.setPower(.75));
        forEachMotor(m -> m.setTargetPosition((int) (WHEEL_MOTOR_ENCODER_SCALING * Tiles)));
        forEachMotor(m -> m.setMode(DcMotor.RunMode.RUN_TO_POSITION));
        sleep(2000);
    }


    // Positive rotates to the left, and negative rotates to the right
    public void rotateDegrees(double Deg) {
        // Negative is left, Positive is right
        imu.resetYaw();

        double power = 0.5;

        forEachMotor(m -> m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));

        if (Deg > 0) {
            leftFront.setPower(-power);
            leftBack.setPower(-power);
            rightFront.setPower(power);
            rightBack.setPower(power);

            while (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < Deg) {
                sleep(10);
            }
        } else {
            leftFront.setPower(power);
            leftBack.setPower(power);
            rightFront.setPower(-power);
            rightBack.setPower(-power);

            while (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > Deg) {
                sleep(10);
            }
        }
        forEachMotor(m -> m.setPower(0));
    }

    public void strafeTiles(float toBeStrafed) {
        forEachMotor(m -> m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        forEachMotor(m -> m.setPower(.75));
        forEachMotor(m -> m.setTargetPosition((int) (WHEEL_MOTOR_ENCODER_SCALING * toBeStrafed)));
        forEachMotor(m -> m.setMode(DcMotor.RunMode.RUN_TO_POSITION));
        sleep(2000);
    }

    private void forEachMotor(Consumer<DcMotor> f){
        Stream.of( leftFront, leftBack, rightBack, rightFront ).forEach(f);

    }
}
