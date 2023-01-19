package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.AutoClass;
import org.firstinspires.ftc.teamcode.Libs.AutoParams;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;

@Autonomous(name = "Auto: Red Terminal", group = "Competition")
public class RedTerminalAuto extends LinearOpMode {
    /*

    OPMODE MAP - PLEASE READ BEFORE EDITING

    This opMode uses TrajectorySequences from RoadRunner. They are made to be run back to back.
    The order of operations is: untilCycle -> firstCycle -> cycles2to4 (repeated 3 times) -> highCycle -> park

    Each parking position is its own TrajectorySequence. They are all made to be run following highCycle.

    All values for target position and heading come from AutoParams.java.

     */

    //lift control init
    public final static HWProfile robot = new HWProfile();
    private LinearOpMode myOpmode=this;
    AutoClass liftControl = new AutoClass(robot,myOpmode);

    //init params
    AutoParams params = new AutoParams();

    //TFOD init
    private static final String TFOD_MODEL_ASSET = "GenericSignalSleeve-Take1.tflite";
    private static final String[] LABELS = {
            "circle",
            "triangle",
            "star"
    };
    private static final String VUFORIA_KEY =
            "AfHl2GP/////AAABmeJc93xOhk1MvZeKbP5E43taYJ6kodzkhsk5wOLGwZI3wxf7v1iTx2Mem/VZSEtpxb3U2fMO7n0EUxSeHRWhOXeX16dMFcjfalezjo3ZkzBuG/y2r4kgLwKs4APyAIClBAon+tf/W/4NkTkYuHGo8zZ0slH/iBpqxvblpNURsG5h4VxPFgF5D/FIfmjnddzQpa4cGarle/Zvuah6q2orUswun31P6ZLuIJvdOIQf7o/ruoRygsSXfVYc35w+Xwm+bwjpZUNzHHYvRNrp0HNWC3Fr2hd0TqWKIIYlCoHj0m5OKX22Ris23V8PdKM/i4/ZIy8JewJXetv1rERC5bfHmUXCS4Rl7RjR+ZscQ5aA0nr8";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    double parkPosition = 2;

    @Override
    public void runOpMode() {
        initVuforia();
        initTfod();
        robot.init(hardwareMap);

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Pose2d startPose= new Pose2d(-params.startPoseX,params.startPoseY);
        drive.setPoseEstimate(startPose);

        TrajectorySequence untilCycle = drive.trajectorySequenceBuilder(startPose)
                //close claw to grab preload
                .addTemporalMarker(0, ()->{
                    liftControl.closeClaw();
                })

                //navigate to mid pole
                .splineTo(new Vector2d(-params.preloadMidX,params.preloadMidY),params.preloadMidHeading/2)

                //lift lift
                .addTemporalMarker(0, ()->{
                    liftControl.moveLiftScore(2);
                })

                //align to mid pole
                .forward(params.preloadMidForward)

                //wait for claw to open
                .addTemporalMarker(0, ()->{
                    liftControl.openClaw();
                })
                .waitSeconds(params.timeOpen)

                //leave main pole
                .back(params.preloadMidBackward)

                //lower lift
                .addTemporalMarker(0, ()->{
                    liftControl.moveLiftScore(0);
                })

                //orient to avoid poles
                .turn(-params.preloadReorientHeading)

                //drive to cone stack
                .splineTo(new Vector2d(-params.preloadWaypointX,params.preloadWaypointY),params.preloadWaypointHeading)

                //raise lift to cone height
                .addTemporalMarker(0, ()->{
                    liftControl.moveLiftGrab();
                })

                //drive to cone stack
                .forward(params.cycle1coneStackForward)

                //grab cone
                .addTemporalMarker(0, ()->{
                    liftControl.closeClaw();
                })
                .waitSeconds(params.timeClose)
                .build();

        TrajectorySequence cycleMid = drive.trajectorySequenceBuilder(untilCycle.end())
                //back away from stack
                .back(params.coneStackReverse)

                //spline to mid pole
                .lineToSplineHeading(new Pose2d(-params.cycleMidX,params.cycleMidY, params.cycleMidHeading))

                //raise lift
                .addTemporalMarker(0, ()->{
                    liftControl.moveLiftScore(2);
                })

                //final pole approach
                .forward(params.cycleMidForward)

                //drop cone
                .addTemporalMarker(0, ()->{
                    liftControl.openClaw();
                })
                .waitSeconds(params.timeOpen)

                //back from pole
                .back(params.cycleMidReverse)

                //lower lift
                .addTemporalMarker(0, ()->{
                    liftControl.moveLiftScore(0);
                })

                //return to stack
                .lineToSplineHeading(new Pose2d(-params.coneStackAlignX,params.coneStackAlignY,params.coneStackAlignHeadingRed))
                .forward(params.coneStackForward)

                //raise lift to cone height
                .addTemporalMarker(0, ()->{
                    liftControl.moveLiftGrab();
                })

                //drive to cone stack
                .forward(params.coneStackForward)

                //grab cone
                .addTemporalMarker(0, ()->{
                    liftControl.closeClaw();
                })
                .waitSeconds(params.timeClose)
                .build();

        TrajectorySequence cycleHigh = drive.trajectorySequenceBuilder(cycleMid.end())
                //back away from stack
                .back(params.coneStackReverse)

                //spline to high pole
                .lineToSplineHeading(new Pose2d(-params.cycleHighX,params.cycleHighY,params.cycleHighHeading))

                //raise lift
                .addTemporalMarker(0.25, ()->{
                    liftControl.moveLiftScore(3);
                })

                //final pole approach
                .forward(params.cycleHighForward)

                //drop cone
                .addTemporalMarker(0, ()->{
                    liftControl.openClaw();
                })
                .waitSeconds(params.timeOpen)

                //back from pole
                .back(params.cycleHighReverse)

                .build();

        //parking position 1
        TrajectorySequence park1= drive.trajectorySequenceBuilder(cycleHigh.end())
                .strafeLeft(params.park36Inch)
                .build();

        //parking position 2 just stays where highCycle ends


        //parking position 3
        TrajectorySequence park3= drive.trajectorySequenceBuilder(cycleHigh.end())
                .strafeRight(params.park12Inch)
                .build();

        while(!opModeIsActive()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Objects Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display image position/size information for each one
                    // Note: "Image number" refers to the randomized image orientation/number
                    for (Recognition recognition : updatedRecognitions) {
                        double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                        double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                        double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                        double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                        telemetry.addData(""," ");
                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                        telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                        telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);


                        if(recognition.getLabel() == "circle"){
                            parkPosition = 1;
                        } else if(recognition.getLabel() == "star" ){
                            parkPosition = 3;
                        } else parkPosition = 2;
                    }
                    telemetry.update();
                }
            }

        }  // end of while

        waitForStart();

        if(isStopRequested()) return;

        //score preload
        drive.followTrajectorySequence(untilCycle);

        //run 4 mid cycles
        for(int i=0;i<2;i++) {
            drive.followTrajectorySequence(cycleMid);
        }

        //run 1 high cycle
        drive.followTrajectorySequence(cycleHigh);

        //parking logic
        if(parkPosition==1){
            drive.followTrajectorySequence(park1);
        }else if(parkPosition==2){

        }else if(parkPosition==3){
            drive.followTrajectorySequence(park3);
        }

    }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }


    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.6f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}
