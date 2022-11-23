package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class CV_Command extends LinearOpMode {
    OpenCvCamera phonecam;

    public void runOpMode() throws InterruptedException
    {
        int webcam = hardwareMap.appContext.getResources().getIdentifier("Webcam", "id", hardwareMap.appContext.getPackageName());
        phonecam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, webcam);
        OpenCV_Test detector = new OpenCV_Test(telemetry);
        phonecam.setPipeline(detector);

        phonecam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                phonecam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        waitForStart();
        phonecam.stopStreaming();
    };
};
