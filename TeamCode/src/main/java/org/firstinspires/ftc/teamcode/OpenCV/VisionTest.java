package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCV.GripPipeline;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="camera", group="Iterative Opmode")
public class VisionTest extends OpMode {
    private OpenCvWebcam webcam;
    private DetectionLevel level;
    private GripPipeline grip;

    private static final int CAMERA_WIDTH = 1280;

    @Override
    public void init() {
        grip = new GripPipeline();


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "pp"), cameraMonitorViewId);

        webcam.setPipeline(grip);
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode){ }
        });
        telemetry.addData("status", "initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        telemetry.update();
    }

    private void updateDetermination(){
        List<MatOfPoint> hulls = grip.findContoursOutput();
        for(int i = 0; i < hulls.size(); i++){
            Rect boundingRect = Imgproc.boundingRect(hulls.get(i));
            if(boundingRect.area() < 20){
                hulls.remove(i);
                i--;
            }
        }

        if(hulls.size() == 0){
            level = DetectionLevel.LEVEL_THREE;
        }else{
            hulls.size();
            double areaSum = 0;
            double xAvg = 0;
            //double yAvg = 0;
            for (int i = 0; i < hulls.size(); i++) {
                Rect boundingRect = Imgproc.boundingRect(hulls.get(i));
                double areaTotal = boundingRect.area();
                areaSum = areaSum + areaTotal;
                xAvg = xAvg + areaTotal * (boundingRect.x + boundingRect.width / 2d);
                //yAvg = yAvg + areaLol*(boundingRect.y + boundingRect.height / 2d);
            }
            xAvg = xAvg / areaSum;

            if (xAvg > CAMERA_WIDTH/2) {
                this.level = DetectionLevel.LEVEL_TWO;
            } else {
                this.level = DetectionLevel.LEVEL_ONE;
            }
        }
    }

    public DetectionLevel currentDetermination(){
        updateDetermination();
        return this.level;
    }

    public void stop(){  }

    public enum DetectionLevel{
        LEVEL_ONE("1 - Lowest level"), LEVEL_TWO("2 - Second level"),
        LEVEL_THREE("3 - Top level"), UNKNOWN("Unknown");

        private final String level;

        DetectionLevel(String str) {
            this.level = str;
        }

        @Override
        public String toString() { return this.level; }
    }
}