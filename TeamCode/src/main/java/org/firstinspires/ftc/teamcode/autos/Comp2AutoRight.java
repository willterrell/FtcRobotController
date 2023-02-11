package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.movement.imu.RotateWithIMU;
import org.firstinspires.ftc.teamcode.movement.roadrunner.HighJunctionToParking;
import org.firstinspires.ftc.teamcode.movement.roadrunner.StartToHighJunction;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.structures.ParkingPosition;
import org.firstinspires.ftc.teamcode.structures.PlaceholderState;
import org.firstinspires.ftc.teamcode.structures.PosType;
import org.firstinspires.ftc.teamcode.structures.Side;
import org.firstinspires.ftc.teamcode.structures.TelemetryObj;
import org.firstinspires.ftc.teamcode.teleops.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.utilities.CenterOnPole.CenterOnPoleState;
import org.firstinspires.ftc.teamcode.utilities.OpenClawState;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
@Config
public class Comp2AutoRight extends LinearOpMode {
    public static double START_X = 34.5, START_Y = 63, START_DEG = 0;
    public static Pose2d START = new Pose2d(START_X, START_Y, Math.toRadians(START_DEG));
    public static boolean TEST = true;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.04;

    //int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    // Tag IDs
    int ID1 = 13;
    int ID2 = 14;
    int ID3 = 15;

    AprilTagDetection tagOfInterest = null;

    ParkingPosition parkingPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        // hardware init
        ElapsedTime timer = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        HardwareHandler hardwareHandler = new HardwareHandler(hardwareMap, new Position(), START_DEG, telemetry);
        hardwareHandler.setDrive(drive);
        drive.setPoseEstimateAndTrajEnd(START);

        // april tags
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

        waitAndDetectTags();

        // abstate init (note inside run)
        StartToHighJunction startToHighJunction = new StartToHighJunction("high", hardwareHandler, Side.RIGHT);
        CenterOnPoleState center1 = new CenterOnPoleState("center1", hardwareHandler, true, true);
        OpenClawState open = new OpenClawState("open", hardwareHandler);
        HighJunctionToParking parking = new HighJunctionToParking("parking", hardwareHandler, parkingPosition, Side.RIGHT);
        RotateWithIMU finalRotate = new RotateWithIMU("align", hardwareHandler, 0, PosType.ABSOLUTE);
//        HighJunctionToConeStack highJunctionToConeStack = new HighJunctionToConeStack("coneStack", drive);
//        CenterOnPoleState center2 = new CenterOnPoleState("center2", hardwareHandler, false, true);
//        ConeStackToHighJunction coneStackToHighJunction = new ConeStackToHighJunction("highJunction2", drive);
//        CenterOnPoleState center3 = new CenterOnPoleState("center3", hardwareHandler, true, true);
        startToHighJunction.putNextState("next", center1);
        center1.putNextState("next", open);
        center1.putNextState("error", parking);
        open.putNextState("next", parking);
        parking.putNextState("next", finalRotate);
        finalRotate.putNextState("next", new PlaceholderState());

//        center1.putNextState("error", new PlaceholderState());
//        highJunctionToConeStack.putNextState("next", center2);
//        center2.putNextState("next", coneStackToHighJunction);
//        coneStackToHighJunction.putNextState("next", center3);
//        center3.putNextState("next", new PlaceholderState());
        AbState currState = startToHighJunction;
        currState.init();

        double prevTime = timer.milliseconds();
        hardwareHandler.closeClaw();
        while (opModeIsActive()) {
            currState.run();
            currState = currState.next();
            drive.update();
            hardwareHandler.updateSlides();
            for (TelemetryObj obj : currState.getTelemetries()) {
                telemetry.addData(obj.getCaption(), obj.getContent());
            }
            telemetry.addData("pose", drive.getPoseEstimate());
            telemetry.addData("r", hardwareHandler.getIMUZAngle());
            telemetry.addData("update", timer.milliseconds() - prevTime);
            prevTime = timer.milliseconds();
            telemetry.update();
        }
    }
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    void waitAndDetectTags() {
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == ID1 || tag.id == ID2 || tag.id == ID3)//ID_TAG_OF_INTEREST)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }


        if(tagOfInterest == null)
        {
            // tag not seen
            parkingPosition = ParkingPosition.ONE;
        } else if(tagOfInterest.id == ID1)
        {
            parkingPosition = ParkingPosition.ONE;
        } else if(tagOfInterest.id == ID2)
        {
            parkingPosition = ParkingPosition.TWO;
        } else if(tagOfInterest.id == ID3)
        {
            parkingPosition = parkingPosition.THREE;
        }
    }
}
