package org.firstinspires.ftc.teamcode.Robot.Motion;

import static android.os.SystemClock.sleep;

public class autoSequence {
    private protoDrive robot;

    // Constructor: Needs a drive object to work
    public autoSequence(protoDrive drive) {
        this.robot = drive;
    }

    // --- FLUENT METHODS (Return 'this') ---

    public autoSequence driveTiles(float tiles) {
        robot.driveTiles(tiles); // Call the dumb hardware method
        return this;             // Return the builder to keep chaining
    }

    public autoSequence rotateDegrees(double degrees) {
        robot.rotateDegrees(degrees);
        return this;
    }
}