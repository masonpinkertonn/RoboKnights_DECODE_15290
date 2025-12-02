package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Vision")
public class Vision extends OpMode {

    VisionPortal visionPortal;
    AprilTagProcessor aprilTagProcessor;

    @Override
    public void init() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(webcamName, aprilTagProcessor);
    }

    @Override
    public void init_loop() {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        StringBuilder idsFound = new StringBuilder();
        for (AprilTagDetection detection : currentDetections)
        {
            idsFound.append(detection.id);
            idsFound.append(" ");
            idsFound.append(detection.ftcPose.x);
            idsFound.append(" ");
            idsFound.append(detection.ftcPose.y);
            idsFound.append(" ");
            idsFound.append(detection.ftcPose.z);
        }
        telemetry.addData("id data", idsFound);
    }

    @Override
    public void loop() {

    }
}
