package org.firstinspires.ftc.teamcode.opModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.Command.CommandScheduler;
import org.firstinspires.ftc.teamcode.util.Controller;
import org.firstinspires.ftc.teamcode.vision.Color.RedSampleLocator;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@TeleOp(name="Purple Vision", group="Vision")
public class redVisionSS extends LinearOpMode {
    OpenCvCamera webcam;
    RedSampleLocator pipeline;

    CommandScheduler scheduler;

    Controller gp1;


    @Override
    public void runOpMode() throws InterruptedException {

        gp1 = new Controller(gamepad1, scheduler);

        scheduler = new CommandScheduler();;

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"));

        pipeline = new RedSampleLocator();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam, 60);

            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if(gp1.a()){
                pipeline.SS = true;
            }

            telemetry.addData("YELLOW SS INFO: " , pipeline.YellowSSInfo);
            telemetry.addLine();
            telemetry.addData("RED SS INFO: " , pipeline.redSSInfo);

            telemetry.update();

        }
    }
}