package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.graphics.Rect;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.Comparator;
import java.util.List;

@TeleOp(name = "Vision Training", group = "Vision")
public class VisionTraining extends LinearOpMode {

    // =========================================================================
    // Hardware
    // =========================================================================
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // =========================================================================
    // Vision
    // =========================================================================
    private VisionPortal visionPortal;
    private ColorBlobLocatorProcessor colorLocator;


    // =========================================================================
    // OpMode
    // =========================================================================

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Motors ---
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- Vision ---
        // Format: ColorSpace, Min Channel 1, Min 2, Min 3, Max Channel 1, Max 2, Max 3
        ColorRange pollen = new ColorRange(
                ColorSpace.HSV,
                new Scalar( 48.6, 79, 94),   // Minimum H, S, V values (0-255 scaled)
                new Scalar( 47.3, 100, 87)   // Maximum H, S, V values (0-255 scaled)
        );


        // https://ftc-docs.firstinspires.org/en/latest/color_processing/color-locator-explore/color-locator-explore.html
        colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setBlurSize(5)
                //.setErodeSize(4)
                .setDrawContours(true)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1.0, 1.0, -1.0))
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(colorLocator)
                .setCameraResolution(new android.util.Size(640, 480))
                .build();

        // wait until camera is ready
        while (!isStopRequested() &&
                visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting for stream...");
            telemetry.update();
            sleep(50);
        }

        telemetry.addData("Status", "Ready — press START");
        telemetry.update();
        waitForStart();

        // =====================================================================
        // MAIN LOOP
        // =====================================================================
        while (opModeIsActive()) {

            telemetry.addData("Status", "OpModeActive");



            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

            // filter blob list
            //ColorBlobLocatorProcessor.Util.filterByArea(200, 20000, blobs);

            // filter by area
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                    200,
                    600,
                    blobs
            );

            // filter by aspect
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_ASPECT_RATIO,
                    .75,
                    1.25,
                    blobs
            );


            if (!blobs.isEmpty()) {
                blobs.sort(Comparator.comparingDouble(ColorBlobLocatorProcessor.Blob::getContourArea).reversed());

                RotatedRect box = blobs.get(0).getBoxFit();

                telemetry.addData("find blobs",  "YES");
                telemetry.addData("number of blobs", blobs.size() );
                telemetry.addData("max blob x location", box.center.x);

                // calc error

                // find the error of the locatoin of the ball and the center of the code
                double error = box.center.x - 320;

                //use the error to set the pivotPower value
                double pivotPower = 0.25 * error;

                //set pivot power based on error
                //setPivotPower( pivotPower );

                if (error>0){
                    pivotPower = 0.3;
                }
                else {
                    pivotPower = -.3;

                }

                setPivotPower( pivotPower );

                telemetry.addData("error", error);
                telemetry.addData("pivot power", pivotPower);
            }
            else{
                telemetry.addData("find blobs",  "no");
                stopDrive();

            }

            telemetry.update();
        }  // end main while loop
    }



    // =========================================================================
    // Drive helpers
    // =========================================================================
    /**
     * Mecanum drive with simultaneous forward + pivot.
     * @param forward  +1.0 = forward
     * @param strafe   +1.0 = strafe right (unused here but wired up)
     * @param pivot    +1.0 = clockwise
     */
    private void setMecanumDrive(double forward, double strafe, double pivot) {
        double fl = forward + strafe + pivot;
        double fr = forward - strafe - pivot;
        double bl = forward - strafe + pivot;
        double br = forward + strafe - pivot;

        // Normalize if any value exceeds 1.0
        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }
    private void stopDrive() {
        setMecanumDrive(0, 0, 0);
    }
    private void setPivotPower(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(-power);
    }


    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior b) {
        frontLeft.setZeroPowerBehavior(b);
        frontRight.setZeroPowerBehavior(b);
        backLeft.setZeroPowerBehavior(b);
        backRight.setZeroPowerBehavior(b);
    }


}  // end VisionTraining Class
