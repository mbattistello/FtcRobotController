package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

@TeleOp(name = "Yellow Ball Tracker", group = "Vision")
public class YellowBallTracker extends LinearOpMode {

    // -------------------------------------------------------------------------
    // Hardware
    // -------------------------------------------------------------------------
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // -------------------------------------------------------------------------
    // Vision
    // -------------------------------------------------------------------------
    private VisionPortal         visionPortal;
    private ColorBlobLocatorProcessor colorLocator;

    // -------------------------------------------------------------------------
    // Tuning Constants
    // -------------------------------------------------------------------------

    /** Width of the USB webcam stream in pixels (standard 640×480). */
    private static final int    FRAME_WIDTH_PX   = 640;

    /** Pixels from center before the robot starts pivoting. Prevents jitter. */
    private static final double CENTER_DEADBAND   = 25.0;

    /** Minimum blob contour area (px²) to consider a valid ball detection. */
    private static final double MIN_BLOB_AREA     = 4000;  // orig 750

    /** Maximum blob area — rejects noise/huge false positives. */
    private static final double MAX_BLOB_AREA     = 80_000.0;

    /**
     * Maximum pivot motor power. Tune this down if the robot overshoots.
     * goBILDA mecanum wheels are grippy — start conservative.
     */
    private static final double MAX_PIVOT_POWER   = 0.45;

    /**
     * Proportional gain for pivot. Power = kP * pixelError.
     * With FRAME_WIDTH=640, an edge-to-edge error of 320px → ~0.45 power.
     */
    private static final double kP                = MAX_PIVOT_POWER / (FRAME_WIDTH_PX / 2.0);

    // -------------------------------------------------------------------------

    @Override
    public void runOpMode() {

        // --- Motor Setup ---------------------------------------------------
        frontLeft  = hardwareMap.get(DcMotor.class, "left_front_drive");
        frontRight = hardwareMap.get(DcMotor.class, "right_front_drive");
        backLeft   = hardwareMap.get(DcMotor.class, "left_back_drive");
        backRight  = hardwareMap.get(DcMotor.class, "right_back_drive");

        // goBILDA mecanum: left side reversed for standard forward drive
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- Vision Setup --------------------------------------------------
        colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.GREEN)          // Target: ColorRange.YELLOW
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setBlurSize(5)                                   // Reduces false positives
                .setDrawContours(true)                            // Overlay on DS preview
                // ROI: full frame in unity-center coords (left, top, right, bottom)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1.0, 1.0, 1.0, -1.0))
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(colorLocator)
                .setCameraResolution(new android.util.Size(640, 480))
                .build();

        // Wait for camera to open before showing "ready"
        while (!isStopRequested() && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
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

            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();




            // Filter out blobs that are too small or too large
            //ColorBlobLocatorProcessor.Util.filterByArea(MIN_BLOB_AREA, MAX_BLOB_AREA, blobs);
            ColorBlobLocatorProcessor.Util.filterByCriteria( ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                                                                MIN_BLOB_AREA,
                                                                MAX_BLOB_AREA,
                                                                blobs);


            // check if blobs found
            if (!blobs.isEmpty()) {

                // Pick the largest blob as the primary target ball
                blobs.sort(Comparator.comparingDouble(
                        ColorBlobLocatorProcessor.Blob::getContourArea).reversed());

                ColorBlobLocatorProcessor.Blob target = blobs.get(0);
                RotatedRect box = target.getBoxFit();

                double ballCenterX   = box.center.x;              // pixels from left edge
                double frameCenterX  = FRAME_WIDTH_PX / 2.0;      // 320.0
                double pixelError    = ballCenterX - frameCenterX; // + = ball is RIGHT of center

                if (Math.abs(pixelError) > CENTER_DEADBAND) {
                    // Proportional pivot: positive error → turn right
                    double pivotPower = kP * pixelError;
                    pivotPower = clamp(pivotPower, -MAX_PIVOT_POWER, MAX_PIVOT_POWER);
                    setPivotPower(pivotPower);

                    telemetry.addData("Action", pixelError > 0 ? "Pivoting RIGHT →" : "Pivoting LEFT ←");
                } else {
                    stopDrive();
                    telemetry.addData("Action", "✓ Centered on ball");
                }

                //telemetry.addData("Ball Center X",  "%.1f px", ballCenterX);
                //telemetry.addData("Pixel Error",    "%.1f px", pixelError);
                //telemetry.addData("Blob Area",      "%.0f px²", target.getContourArea());

                telemetry.addData("Ball Center X",   ballCenterX);
                telemetry.addData("Pixel Error",   pixelError);
                telemetry.addData("Blob Area",     target.getContourArea());
                telemetry.addData("Blobs Found",    blobs.size() );

            }
            else {
                //no blobs found
                stopDrive();
                telemetry.addData("Action",      "Searching — no yellow ball detected");
                telemetry.addData("Blobs Found", 0);
            }

            telemetry.update();
        }  // while loop

        // Cleanup
        stopDrive();
        visionPortal.close();
    }

    // =========================================================================
    // Drive Helpers
    // =========================================================================

    /**
     * Pivot in place using mecanum wheels.
     * Positive power = clockwise (right), negative = counter-clockwise (left).
     *
     * Mecanum pivot:
     *   Left  side: +power  (forward)
     *   Right side: -power  (backward)
     */
    private void setPivotPower(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(-power);
    }

    private void stopDrive() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        frontLeft.setZeroPowerBehavior(behavior);
        frontRight.setZeroPowerBehavior(behavior);
        backLeft.setZeroPowerBehavior(behavior);
        backRight.setZeroPowerBehavior(behavior);
    }

    // =========================================================================
    // Utility
    // =========================================================================
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
