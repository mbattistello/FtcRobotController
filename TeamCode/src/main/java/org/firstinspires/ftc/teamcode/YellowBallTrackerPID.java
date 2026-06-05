package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import org.opencv.core.RotatedRect;

import java.util.Comparator;
import java.util.List;

@TeleOp(name = "Yellow Ball Tracker PID", group = "Vision")
public class YellowBallTrackerPID extends LinearOpMode {

    // =========================================================================
    // PID Controller — reusable inner class
    // =========================================================================
    private static class PIDController {

        // Gains — tuned per controller instance
        private double kP, kI, kD;

        // State
        private double integralSum   = 0;
        private double lastError     = 0;
        private double lastOutput    = 0;
        private boolean firstRun     = true;

        // Anti-windup: clamp integral contribution
        private double integralLimit = 1.0;

        // Output clamp
        private double outputMin = -1.0;
        private double outputMax =  1.0;

        private final ElapsedTime timer = new ElapsedTime();

        public PIDController(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }

        public PIDController setOutputBounds(double min, double max) {
            this.outputMin = min;
            this.outputMax = max;
            return this;
        }

        public PIDController setIntegralLimit(double limit) {
            this.integralLimit = limit;
            return this;
        }

        /**
         * Call once per loop. Returns the PID output for the given error.
         * @param error  setpoint - measurement  (positive = above target)
         */
        public double calculate(double error) {
            double dt = timer.seconds();
            timer.reset();

            // Skip derivative spike on very first call
            if (firstRun) {
                lastError = error;
                firstRun  = false;
                dt        = 0.02; // assume 20 ms first frame
            }

            // Clamp dt to avoid huge D spikes if loop stalls
            dt = Math.max(dt, 0.005);

            // --- P ---
            double pTerm = kP * error;

            // --- I --- (with anti-windup clamp)
            integralSum += error * dt;
            integralSum  = clamp(integralSum, -integralLimit, integralLimit);
            double iTerm = kI * integralSum;

            // --- D ---
            double derivative = (dt > 0) ? (error - lastError) / dt : 0;
            double dTerm      = kD * derivative;

            lastError  = error;
            lastOutput = clamp(pTerm + iTerm + dTerm, outputMin, outputMax);
            return lastOutput;
        }

        /** Reset integral and derivative state (call when switching states). */
        public void reset() {
            integralSum = 0;
            lastError   = 0;
            firstRun    = true;
            timer.reset();
        }

        public double getLastOutput()   { return lastOutput;   }
        public double getIntegralSum()  { return integralSum;  }

        public void setGains(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }

        private static double clamp(double v, double min, double max) {
            return Math.max(min, Math.min(max, v));
        }
    }

    // =========================================================================
    // Drive State Machine
    // =========================================================================
    private enum DriveState {
        SEARCH,      // No ball — spin scan
        CENTERING,   // Ball visible — pivot to align
        APPROACH,    // Aligned — drive toward ball
        COLLECT      // Close enough — run intake
    }

    private DriveState currentState = DriveState.SEARCH;

    // =========================================================================
    // Hardware
    // =========================================================================
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // =========================================================================
    // Vision
    // =========================================================================
    private VisionPortal              visionPortal;
    private ColorBlobLocatorProcessor colorLocator;

    // =========================================================================
    // Tuning Constants
    // =========================================================================

    // --- Frame ---
    private static final int    FRAME_WIDTH_PX  = 640;
    private static final double FRAME_CENTER_X  = FRAME_WIDTH_PX / 2.0; // 320 px

    // --- Blob filters ---
    private static final double MIN_BLOB_AREA   = 750.0;
    private static final double MAX_BLOB_AREA   = 80_000.0;

    // --- Pivot PID (pixel error → turn power) ---
    //   Error units : pixels  (range ≈ ±320)
    //   Output units: motor power (−1.0 to +1.0)
    //
    //   Start with kI = 0, kD = 0 and tune kP first.
    //   Add kD to dampen oscillation. Add a tiny kI only if there's
    //   a persistent steady-state offset you can't fix with kP alone.
    private static final double PIVOT_kP = 0.0014;   // ~0.45 power at 320px error
    private static final double PIVOT_kI = 0.0002;   // small — anti-drift
    private static final double PIVOT_kD = 0.00008;  // dampen overshoot

    // --- Approach PID (area error → forward power) ---
    //   Error units : px²   (target area − current area)
    //   Output units: motor power (0.0 to 0.5)
    //
    //   Positive error → ball is far → drive forward.
    //   When error ≤ 0 the approach PID output is clamped to 0.
    private static final double APPROACH_kP = 0.000015;
    private static final double APPROACH_kI = 0.000001;
    private static final double APPROACH_kD = 0.000005;

    // --- Setpoints ---
    private static final double COLLECT_AREA    = 18_000.0; // px² → trigger intake
    private static final double CENTER_DEADBAND =    20.0;  // px  → "close enough"
    private static final double AREA_DEADBAND   = 1_000.0;  // px² → "close enough"

    // --- Search ---
    private static final double SEARCH_POWER    = 0.25;
    private static final double SEARCH_TIMEOUT  = 3.5;     // seconds before reversing spin

    // =========================================================================
    // PID instances
    // =========================================================================
    private final PIDController pivotPID = new PIDController(PIVOT_kP, PIVOT_kI, PIVOT_kD)
            .setOutputBounds(-0.55, 0.55)
            .setIntegralLimit(150);   // cap integral at ±150 px·s

    private final PIDController approachPID = new PIDController(APPROACH_kP, APPROACH_kI, APPROACH_kD)
            .setOutputBounds(0.0, 0.50)  // only drive forward, never reverse
            .setIntegralLimit(5_000);    // cap integral at ±5000 px²·s

    // =========================================================================
    // Loop state
    // =========================================================================
    private final ElapsedTime searchTimer  = new ElapsedTime();
    private final ElapsedTime collectTimer = new ElapsedTime();
    private int  ballCount                 = 0;
    private int  searchDirection           = 1; // +1 = CW, −1 = CCW

    // =========================================================================

    @Override
    public void runOpMode() {

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
        colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setBlurSize(5)
                .setDrawContours(true)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1.0, 1.0, 1.0, -1.0))
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(colorLocator)
                .setCameraResolution(new android.util.Size(640, 480))
                .build();

        while (!isStopRequested() &&
                visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting for stream...");
            telemetry.update();
            sleep(50);
        }

        telemetry.addData("Status", "Ready — press START");
        telemetry.update();
        waitForStart();

        searchTimer.reset();

        // =====================================================================
        // MAIN LOOP
        // =====================================================================
        while (opModeIsActive()) {

            // --- Get blobs ---
            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
            ColorBlobLocatorProcessor.Util.filterByArea(MIN_BLOB_AREA, MAX_BLOB_AREA, blobs);
            blobs.sort(Comparator.comparingDouble(
                    ColorBlobLocatorProcessor.Blob::getContourArea).reversed());

            boolean ballVisible = !blobs.isEmpty();

            // --- State transitions ---
            if (!ballVisible && currentState != DriveState.SEARCH && currentState != DriveState.COLLECT) {
                transitionTo(DriveState.SEARCH);
            }

            // --- State execution ---
            switch (currentState) {

                // -----------------------------------------------------------------
                case SEARCH:
                    // Reverse spin direction each timeout cycle to sweep the field
                    if (searchTimer.seconds() > SEARCH_TIMEOUT) {
                        searchDirection *= -1;
                        searchTimer.reset();
                    }
                    setPivotPower(SEARCH_POWER * searchDirection);

                    if (ballVisible) {
                        transitionTo(DriveState.CENTERING);
                    }
                    break;

                // -----------------------------------------------------------------
                case CENTERING: {
                    RotatedRect box   = blobs.get(0).getBoxFit();
                    double pixelError = box.center.x - FRAME_CENTER_X; // + = ball right of center

                    double pivotOut = pivotPID.calculate(-pixelError); // negate: right error → CW pivot
                    setPivotPower(pivotOut);

                    if (Math.abs(pixelError) <= CENTER_DEADBAND) {
                        transitionTo(DriveState.APPROACH);
                    }

                    telemetryPID("Pivot PID", -pixelError, pivotOut);
                    break;
                }

                // -----------------------------------------------------------------
                case APPROACH: {
                    RotatedRect box   = blobs.get(0).getBoxFit();
                    double pixelError = box.center.x - FRAME_CENTER_X;
                    double area       = blobs.get(0).getContourArea();
                    double areaError  = COLLECT_AREA - area;         // shrinks as you close in

                    // Pivot correction while driving (gentler gain mid-approach)
                    double pivotOut   = pivotPID.calculate(-pixelError) * 0.5;

                    // Forward speed from approach PID
                    double forwardOut = (areaError > 0)
                            ? approachPID.calculate(areaError)
                            : 0.0;

                    setMecanumDrive(forwardOut, 0, pivotOut);

                    if (area >= COLLECT_AREA - AREA_DEADBAND) {
                        transitionTo(DriveState.COLLECT);
                    }

                    telemetryPID("Pivot PID",    -pixelError, pivotOut);
                    telemetryPID("Approach PID",  areaError,  forwardOut);
                    telemetry.addData("Ball Area", "%.0f / %.0f px²", area, COLLECT_AREA);
                    break;
                }

                // -----------------------------------------------------------------
                case COLLECT:
                    stopDrive();
                    // Run intake here: intakeMotor.setPower(1.0);

                    if (collectTimer.seconds() > 1.0) {
                        ballCount++;
                        pivotPID.reset();
                        approachPID.reset();
                        transitionTo(DriveState.SEARCH);
                    }
                    break;
            }

            // --- Telemetry ---
            telemetry.addData("═══ STATE",  currentState);
            telemetry.addData("Balls",      ballCount);
            telemetry.addData("Blobs",      blobs.size());
            telemetry.addData("Search Dir", searchDirection > 0 ? "→ CW" : "← CCW");
            telemetry.update();
        }

        stopDrive();
        visionPortal.close();
    }

    // =========================================================================
    // State Machine Helper
    // =========================================================================
    private void transitionTo(DriveState next) {
        // Reset PIDs and timers on state entry
        switch (next) {
            case SEARCH:
                stopDrive();
                pivotPID.reset();
                approachPID.reset();
                searchTimer.reset();
                break;
            case CENTERING:
                stopDrive();
                pivotPID.reset();
                approachPID.reset();
                break;
            case APPROACH:
                approachPID.reset();
                break;
            case COLLECT:
                stopDrive();
                collectTimer.reset();
                break;
        }
        currentState = next;
    }

    // =========================================================================
    // Drive Helpers
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

    private void setPivotPower(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(-power);
    }

    private void stopDrive() {
        setMecanumDrive(0, 0, 0);
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior b) {
        frontLeft.setZeroPowerBehavior(b);
        frontRight.setZeroPowerBehavior(b);
        backLeft.setZeroPowerBehavior(b);
        backRight.setZeroPowerBehavior(b);
    }

    // =========================================================================
    // Telemetry Helper
    // =========================================================================
    private void telemetryPID(String label, double error, double output) {
        telemetry.addData(label + " err", "%.2f", error);
        telemetry.addData(label + " out", "%.3f", output);
    }
}
