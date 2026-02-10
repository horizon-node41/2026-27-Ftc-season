package org.firstinspires.ftc.teamcode.Robot.Motion;

import static android.os.SystemClock.sleep;

import org.firstinspires.ftc.teamcode.Robot.drive;

public class autoSequence {
    private drive robot;

    // Constructor: Needs a drive object to work
    public autoSequence(drive drive) {
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

    // You can add special "Sequence only" methods here too
    public autoSequence wait(long milliseconds) {
        sleep(milliseconds);
        return this;
    }
}