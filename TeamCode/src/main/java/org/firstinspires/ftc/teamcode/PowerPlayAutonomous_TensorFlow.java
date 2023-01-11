package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.ArrayList;
import java.lang.Math;
import java.util.Random;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.util.Range;
import java.io.File;
import java.text.SimpleDateFormat;
import java.util.Stack;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;

@Autonomous(name = "Power Play Autonomous - Tensor Flow", group = "Concept")
public class PowerPlayAutonomous_TensorFlow extends LinearOpMode {
    // Control Hub
    /// Wheel Motors
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;

    /// Claw
    private DcMotor liftMotor = null;
    private CRServo clawServo = null;
    private CRServo rotateServo = null;

    /// IMU
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private float currentAngleX, currentAngleY, currentAngleZ;
    private float zeroAngleX, zeroAngleY, zeroAngleZ;
    private float finalRotAngle = 64.18f;

    /// Lights
    private RevBlinkinLedDriver ledLights;

    // Other
    private int driveDistance, turnDirection;
    private int step = 0;
    private int beganSmoothTravel = 0;
    private int parkingPosition = 0;
    private long initTime;
    private double desiredTime, smoothSpeed;

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    private static final String TFOD_MODEL_ASSET = "bok2electricboogaloo.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

    private static final String[] LABELS =
    {
      "Green", "Pink", "Yellow"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AX54Lyj/////AAABmSsIALipi0y4oiZBAoZS4o4Jppp+qbLTWgVQVVuyveVi7sLhVC8XAwvTGDzKpxm1tiMRMLgYEV3Y5YXvqKMiA7R7TUZQcZeyL9MMGoqcq7rIeFMX01KOuZUmfs754hgbnsINn38JjhLLAH3g2GuKF9QZBF/CJqw/UFKKzR8bDlv4TkkTP8AyxvF9Vyv9G9gQhK2HoOWuSCWQHzIWl+op5LEPLXU7RmdrWzxDm1zEY3DZoax5pYLMRR349NoNzpUFBzwNu+nmEzT3eXQqtppz/vE/gHA0LRys9MAktPmeXQfvaS2YUi4UdE4PcFxfCUPuWe6L9xOQmUBE7hB39jTRkYxGADmTxILyBZB6fD3qyFHv";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    private double smoothDecel (double desiredTime, double speed, long initialTime)
    {
        long currentTime = System.currentTimeMillis() - initialTime;
        return (speed/desiredTime) * Math.exp(-currentTime/desiredTime);
    }

    private double smoothAccel (double desiredTime, double speed, long initialTime)
    {
        long currentTime = System.currentTimeMillis() - initialTime;
        return speed * (1 - Math.exp(-currentTime/desiredTime));
    }

    @Override
    public void runOpMode() {
        // Initialize connection to motors
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        clawServo = hardwareMap.crservo.get("clawServo");
        rotateServo = hardwareMap.crservo.get("rotateServo");

        ledLights = hardwareMap.get(RevBlinkinLedDriver.class, "ledLights");
        ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);

        // Set direction to the motors (may need to change depending on orientation of robot)
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Reset the encoder count - initialize to 0
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // IMU settings
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.7, 16.0/9.0);
        }

        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        /** Wait for the game to begin */
        telemetry.addLine("Working.");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            zeroAngleX = lastAngles.firstAngle;
            zeroAngleY = lastAngles.secondAngle;
            zeroAngleZ = lastAngles.thirdAngle;

            long initTime = System.currentTimeMillis();
            long currentTime = 0;

            while (opModeIsActive()) {
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

                            if (parkingPosition == 0)
                            {
                                if (recognition.getLabel() == "Pink")
                                {
                                    parkingPosition = 1;
                                    ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                                }
                                else if (recognition.getLabel() == "Yellow")
                                {
                                    parkingPosition = 2;
                                    ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
                                }
                                else
                                {
                                    parkingPosition = 3;
                                    ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                                }
                            }
                        }
                        currentTime = System.currentTimeMillis() - initTime;
                        telemetry.addData("Time is", currentTime);

                        if (parkingPosition == 0 && currentTime >= 3000)
                        {
                            parkingPosition = 4;
                            ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GRAY);
                        }

                        telemetry.update();

                        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                        currentAngleX = lastAngles.firstAngle - zeroAngleX;
                        currentAngleY = lastAngles.secondAngle - zeroAngleY;
                        currentAngleZ = lastAngles.thirdAngle - zeroAngleZ;

                        // Telemetry outputs
                        telemetry.addData("X Rotation", currentAngleX);
                        telemetry.addData("Y Rotation", currentAngleY);
                        telemetry.addData("Z Rotation", currentAngleZ);
                        telemetry.addData("Current Step", step);
                        telemetry.addData("Parking Position", parkingPosition);
                        telemetry.addLine("\nMotors:");
                        telemetry.addData("Front Left", frontLeftMotor.getCurrentPosition());
                        telemetry.addData("Front Right", frontRightMotor.getCurrentPosition());
                        telemetry.addData("Back Left", backLeftMotor.getCurrentPosition());
                        telemetry.addData("Back Right", backRightMotor.getCurrentPosition());
                        telemetry.addData("Lift", liftMotor.getCurrentPosition());
                        telemetry.addLine("\nServos");
                        telemetry.addData("Claw", clawServo.getPower());
                        telemetry.addData("Rotation", rotateServo.getPower());
                        telemetry.addLine(""); // blank space to create gap between final of this output and TensorFlow stuff

                        // Set power to motors
                        frontLeftMotor.setPower(0.32);
                        frontRightMotor.setPower(0.32);
                        backLeftMotor.setPower(0.32);
                        backRightMotor.setPower(0.32);
                        liftMotor.setPower(0.5);

                        // -----------------------------
                        // 45.5 counts per inch
                        // Actual Autonomous begins here

                        if (parkingPosition != 0 && step == 0)
                        {
                            step++;
                        }

                        if (step == 1) // Move forward
                        {
                            driveDistance = 1800;

                            frontLeftMotor.setTargetPosition(driveDistance);
                            frontRightMotor.setTargetPosition(driveDistance);
                            backLeftMotor.setTargetPosition(driveDistance);
                            backRightMotor.setTargetPosition(driveDistance);

                            // test number, do calculations
                            liftMotor.setTargetPosition(3400);

                            rotateServo.setPower(0.5);
                            clawServo.setPower(0.52);

                            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                            if (frontLeftMotor.getCurrentPosition() > (driveDistance - 5) && frontRightMotor.getCurrentPosition() > (driveDistance - 5))
                            {
                                frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                                frontLeftMotor.setPower(0.0);
                                frontRightMotor.setPower(0.0);
                                backLeftMotor.setPower(0.0);
                                backRightMotor.setPower(0.0);

                                /* frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); */

                                sleep(250);
                                step++;
                            }
                        }

                        if (step == 2) // Turn left 90 degrees
                        {
                            finalRotAngle = 75f;

                            frontLeftMotor.setPower(-0.32);
                            frontRightMotor.setPower(0.32);
                            backLeftMotor.setPower(-0.32);
                            backRightMotor.setPower(0.32);

                            frontLeftMotor.setTargetPosition(-750);
                            frontRightMotor.setTargetPosition(750);
                            backLeftMotor.setTargetPosition(-750);
                            backRightMotor.setTargetPosition(750);

                            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                            if (frontLeftMotor.getCurrentPosition() < -745 && frontRightMotor.getCurrentPosition() > 745)
                            {
                                frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                                frontLeftMotor.setPower(0.0);
                                frontRightMotor.setPower(0.0);
                                backLeftMotor.setPower(0.0);
                                backRightMotor.setPower(0.0);

                                /*sleep(250);*/
                                step++;
                            }

                            /* if (currentAngleY >= finalRotAngle)
                            {
                                frontLeftMotor.setPower(0.0);
                                frontRightMotor.setPower(0.0);
                                backLeftMotor.setPower(0.0);
                                backRightMotor.setPower(0.0);

                                frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                                rotateServo.setPower(-0.7);

                                sleep(300);
                                clawServo.setPower(0.1);

                                step++;
                            } */
                        }

                        if (step == 3) // Dunk and drop cone
                        {
                            frontLeftMotor.setPower(0.32);
                            frontRightMotor.setPower(0.32);
                            backLeftMotor.setPower(0.32);
                            backRightMotor.setPower(0.32);

                            frontLeftMotor.setTargetPosition(67);
                            frontRightMotor.setTargetPosition(67);
                            backLeftMotor.setTargetPosition(67);
                            backRightMotor.setTargetPosition(67);

                            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                            if (frontLeftMotor.getCurrentPosition() > 62 && frontRightMotor.getCurrentPosition() > 62)
                            {
                                frontLeftMotor.setPower(0.0);
                                frontRightMotor.setPower(0.0);
                                backLeftMotor.setPower(0.0);
                                backRightMotor.setPower(0.0);

                                frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                                rotateServo.setPower(-0.7);

                                sleep(300);
                                clawServo.setPower(0.1);

                                sleep(250);
                                step++;
                            }
                        }

                        if (step == 4) // Reset claw position
                        {
                            sleep(500);
                            clawServo.setPower(0.52);

                            sleep(250);
                            step++;
                        }

                        if (step == 5) // Pan over a smidgeon
                        {


                            frontLeftMotor.setTargetPosition(-630);
                            frontRightMotor.setTargetPosition(630);
                            backLeftMotor.setTargetPosition(630);
                            backRightMotor.setTargetPosition(-630);

                            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                            frontLeftMotor.setPower(-0.32);
                            frontRightMotor.setPower(0.32);
                            backLeftMotor.setPower(0.32);
                            backRightMotor.setPower(-0.32);

                            if (frontLeftMotor.getCurrentPosition() < -625 && frontRightMotor.getCurrentPosition() > 625)
                            {
                                frontLeftMotor.setPower(0.0);
                                frontRightMotor.setPower(0.0);
                                backLeftMotor.setPower(0.0);
                                backRightMotor.setPower(0.0);

                                rotateServo.setPower(0.5);

                                frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                                sleep(250);
                                step++;
                            }
                        }

                        if (step == 6) // Make a sick 180 to get ready to pick up a cone
                        {

                            frontLeftMotor.setPower(-0.32);
                            frontRightMotor.setPower(0.32);
                            backLeftMotor.setPower(-0.32);
                            backRightMotor.setPower(0.32);

                            frontLeftMotor.setTargetPosition(-1570);
                            frontRightMotor.setTargetPosition(1570);
                            backLeftMotor.setTargetPosition(-1570);
                            backRightMotor.setTargetPosition(1570);

                            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                            liftMotor.setPower(-0.5);
                            liftMotor.setTargetPosition(1000);

                            rotateServo.setPower(-0.1);
                            clawServo.setPower(0.1);

                            if (frontLeftMotor.getCurrentPosition() < -1565 && frontRightMotor.getCurrentPosition() > 1565)
                            {
                                frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                                frontLeftMotor.setPower(0.0);
                                frontRightMotor.setPower(0.0);
                                backLeftMotor.setPower(0.0);
                                backRightMotor.setPower(0.0);

                                sleep(250);
                                step++;
                            }
                        }
                        if (step == 7) // 180 turn correction
                        {
                            /*if (currentAngleY < 89.5)
                            {
                                frontLeftMotor.setPower(-0.07);
                                frontRightMotor.setPower(0.07);
                                backLeftMotor.setPower(-0.07);
                                backRightMotor.setPower(0.07);

                                frontLeftMotor.setTargetPosition(-10);
                                frontRightMotor.setTargetPosition(10);
                                backLeftMotor.setTargetPosition(-10);
                                backRightMotor.setTargetPosition(10);

                                frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            }
                            else if (currentAngleY > 90.5)
                            {
                                frontLeftMotor.setPower(0.07);
                                frontRightMotor.setPower(-0.07);
                                backLeftMotor.setPower(0.07);
                                backRightMotor.setPower(-0.07);

                                frontLeftMotor.setTargetPosition(10);
                                frontRightMotor.setTargetPosition(-10);
                                backLeftMotor.setTargetPosition(10);
                                backRightMotor.setTargetPosition(-10);

                                frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            }
                            else
                            {
                                frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                                step++;
                            }*/
                            step++;
                        }

                        if (step == 8) // moves forward and picks up cone
                        {
                            frontLeftMotor.setPower(0.32);
                            frontRightMotor.setPower(0.32);
                            backLeftMotor.setPower(0.32);
                            backRightMotor.setPower(0.32);

                            frontLeftMotor.setTargetPosition(1150);
                            frontRightMotor.setTargetPosition(1150);
                            backLeftMotor.setTargetPosition(1150);
                            backRightMotor.setTargetPosition(1150);

                            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                            if (frontLeftMotor.getCurrentPosition() > 1095 && frontRightMotor.getCurrentPosition() > 1095)
                            {
                                frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                                frontLeftMotor.setPower(0.0);
                                frontRightMotor.setPower(0.0);
                                backLeftMotor.setPower(0.0);
                                backRightMotor.setPower(0.0);

                                clawServo.setPower(0.52);
                                sleep(500);
                                rotateServo.setPower(0.5);

                                sleep(250);
                                step++;
                            }
                        }

                        if (step == 9) // Backs up to align with low junction
                        {
                            frontLeftMotor.setPower(-0.32);
                            frontRightMotor.setPower(-0.32);
                            backLeftMotor.setPower(-0.32);
                            backRightMotor.setPower(-0.32);

                            frontLeftMotor.setTargetPosition(-1050);
                            frontRightMotor.setTargetPosition(-1050);
                            backLeftMotor.setTargetPosition(-1050);
                            backRightMotor.setTargetPosition(-1050);

                            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                            if (frontLeftMotor.getCurrentPosition() < -1045 && frontRightMotor.getCurrentPosition() < -1045)
                            {
                                frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                                frontLeftMotor.setPower(0.0);
                                frontRightMotor.setPower(0.0);
                                backLeftMotor.setPower(0.0);
                                backRightMotor.setPower(0.0);

                                sleep(250);
                                step++;
                            }
                        }

                        if (step == 10) // pan to backtrack to step 5, but robot orientation is the same as step 7
                        {
                            frontLeftMotor.setPower(-0.32);
                            frontRightMotor.setPower(0.32);
                            backLeftMotor.setPower(0.32);
                            backRightMotor.setPower(-0.32);

                            frontLeftMotor.setTargetPosition(-630);
                            frontRightMotor.setTargetPosition(630);
                            backLeftMotor.setTargetPosition(630);
                            backRightMotor.setTargetPosition(-630);

                            liftMotor.setTargetPosition(1500);

                            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                            if (frontLeftMotor.getCurrentPosition() < -625 && frontRightMotor.getCurrentPosition() > 625)
                            {
                                frontLeftMotor.setPower(0.0);
                                frontRightMotor.setPower(0.0);
                                backLeftMotor.setPower(0.0);
                                backRightMotor.setPower(0.0);

                                rotateServo.setPower(0.5);

                                frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                                sleep(250);
                                step++;
                            }
                        }

                        if (step == 11) // get ready to DUNK
                        {
                            frontLeftMotor.setPower(0.32);
                            frontRightMotor.setPower(0.32);
                            backLeftMotor.setPower(0.32);
                            backRightMotor.setPower(0.32);

                            frontLeftMotor.setTargetPosition(100);
                            frontRightMotor.setTargetPosition(100);
                            backLeftMotor.setTargetPosition(100);
                            backRightMotor.setTargetPosition(100);

                            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                            if (frontLeftMotor.getCurrentPosition() > 95 && frontRightMotor.getCurrentPosition() > 95)
                            {
                                frontLeftMotor.setPower(0.0);
                                frontRightMotor.setPower(0.0);
                                backLeftMotor.setPower(0.0);
                                backRightMotor.setPower(0.0);

                                frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                                rotateServo.setPower(-0.1);

                                sleep(300);
                                clawServo.setPower(0.1);

                                sleep(250);
                                step++;
                            }
                        }

                        if (step == 12) // return to base
                        {
                            frontLeftMotor.setPower(0.32);
                            frontRightMotor.setPower(-0.32);
                            backLeftMotor.setPower(-0.32);
                            backRightMotor.setPower(0.32);

                            frontLeftMotor.setTargetPosition(600);
                            frontRightMotor.setTargetPosition(-600);
                            backLeftMotor.setTargetPosition(-600);
                            backRightMotor.setTargetPosition(600);

                            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                            if (frontLeftMotor.getCurrentPosition() > 595 && frontRightMotor.getCurrentPosition() < -595)
                            {
                                frontLeftMotor.setPower(0.0);
                                frontRightMotor.setPower(0.0);
                                backLeftMotor.setPower(0.0);
                                backRightMotor.setPower(0.0);

                                frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                                rotateServo.setPower(0.5);
                                clawServo.setPower(0.52);
                                liftMotor.setTargetPosition(0);
                                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                                sleep(250);
                                step++;
                            }
                        }

                        if (step == 13 && parkingPosition == 3) // K, drive forward
                        {
                            frontLeftMotor.setPower(0.32);
                            frontRightMotor.setPower(0.32);
                            backLeftMotor.setPower(0.32);
                            backRightMotor.setPower(0.32);

                            frontLeftMotor.setTargetPosition(900);
                            frontRightMotor.setTargetPosition(900);
                            backLeftMotor.setTargetPosition(900);
                            backRightMotor.setTargetPosition(900);

                            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                            if (frontLeftMotor.getCurrentPosition() > 895 && frontRightMotor.getCurrentPosition() > 895)
                            {
                                frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                                frontLeftMotor.setPower(0.0);
                                frontRightMotor.setPower(0.0);
                                backLeftMotor.setPower(0.0);
                                backRightMotor.setPower(0.0);

                                sleep(250);
                                step++;
                            }
                        }

                        if (step == 13 && (parkingPosition == 1 || parkingPosition == 4)) // B, drive backward
                        {
                            frontLeftMotor.setPower(-0.32);
                            frontRightMotor.setPower(-0.32);
                            backLeftMotor.setPower(-0.32);
                            backRightMotor.setPower(-0.32);

                            frontLeftMotor.setTargetPosition(-1150);
                            frontRightMotor.setTargetPosition(-1150);
                            backLeftMotor.setTargetPosition(-1150);
                            backRightMotor.setTargetPosition(-1150);

                            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                            if (frontLeftMotor.getCurrentPosition() < -1145 && frontRightMotor.getCurrentPosition() < -1145)
                            {
                                frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                                frontLeftMotor.setPower(0.0);
                                frontRightMotor.setPower(0.0);
                                backLeftMotor.setPower(0.0);
                                backRightMotor.setPower(0.0);

                                sleep(250);
                                step++;
                            }
                        }
                    }
                }
            }
            // Stop function - cancel all power to the motors
            frontLeftMotor.setPower(0.0);
            frontRightMotor.setPower(0.0);
            backLeftMotor.setPower(0.0);
            backRightMotor.setPower(0.0);
            liftMotor.setPower(0.0);
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

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
        tfodParameters.minResultConfidence = 0.5f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}
