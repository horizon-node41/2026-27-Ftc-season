package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

public class Vision {
    // --- PRIVATE: Internal "Complicated" Stuff ---
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    /** * PUBLIC: Initialize the camera.
     * Hides all the builder logic from the main code.
     */
    public void init(HardwareMap hwMap, String webcamName) {
        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, webcamName))
                .addProcessor(aprilTag)
                .build();
    }

    /** * PUBLIC: Get the ID of the first AprilTag seen.
     * Returns -1 if nothing is found.
     */
    public int getDetectedTagID() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        if (currentDetections.size() > 0) {
            return currentDetections.get(0).id;
        }
        return -1;
    }

    /** * PUBLIC: Clean up.
     * Important to stop the camera when not in use to save CPU.
     */
    public void close() {
        visionPortal.close();
    }
}