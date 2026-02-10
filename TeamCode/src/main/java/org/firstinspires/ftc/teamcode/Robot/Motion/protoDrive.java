package org.firstinspires.ftc.teamcode.Robot.Motion;

import static android.os.SystemClock.sleep;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.function.Consumer;
import java.util.stream.Stream;

public class    protoDrive {
    private final HardwareMap hMap;
    private final DcMotor leftFront, leftBack, rightBack, rightFront;
    private final IMU imu;

    static final float WHEEL_MOTOR_ENCODER_SCALING = 1003.046f;

    public protoDrive(HardwareMap hardwareMap) {
        this.hMap = hardwareMap;
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

    // --- Fluent Movement Methods ---

    public protoDrive driveTiles(float tiles) {
        int target = (int) (WHEEL_MOTOR_ENCODER_SCALING * tiles);
        executePositionMove(target);
        sleep(2000);
        return this; // This allows the chaining
    }

    public protoDrive strafeTiles(float toBeStrafed) {
        // Note: For a real mecanum strafe, you'd usually flip motor directions here
        // But keeping your logic:
        int target = (int) (WHEEL_MOTOR_ENCODER_SCALING * toBeStrafed);
        executePositionMove(target);
        sleep(2000);
        return this; // This allows the chaining
    }

    public protoDrive rotateDegrees(double deg) {
        imu.resetYaw();
        double power = 0.5;
        forEachMotor(m -> m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));

        if (deg > 0) {
            setPower(-power, -power, power, power);
            while (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < deg) { sleep(10); }
        } else {
            setPower(power, power, -power, -power);
            while (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > deg) { sleep(10); }
        }

        stopMotors();
        return this; // This allows the chaining
    }

    // --- Helper Methods ---

    private void executePositionMove(int target) {
        forEachMotor(m -> {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setTargetPosition(target);
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(0.75);
        });
    }

    private void setPower(double lf, double lb, double rb, double rf) {
        leftFront.setPower(lf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);
        rightFront.setPower(rf);
    }

    public protoDrive stopMotors() {
        forEachMotor(m -> m.setPower(0));
        return this;
    }

    private void forEachMotor(Consumer<DcMotor> f) {
        Stream.of(leftFront, leftBack, rightBack, rightFront).forEach(f);
    }
}