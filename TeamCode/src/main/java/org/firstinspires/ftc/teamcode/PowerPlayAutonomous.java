package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.List;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;


/*@Autonomous*/
@Autonomous(name="Power Play Auto", group="Interactive Opmode")
@Disabled
public class PowerPlayAutonomous extends OpMode
{

    // Control Hub:v
    // drive motors:
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;

    // claw motor/servos:
    private DcMotor liftMotor = null;
    private CRServo clawServo = null;
    private CRServo rotateServo = null;

    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private float zeroAngle, currentAngle, finalRotAngleM2;


    @Override

    public void init ()
    {
        // Initialize connection to motors
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        clawServo = hardwareMap.crservo.get("clawServo");
        rotateServo = hardwareMap.crservo.get("rotateServo");



        // Set direction to the motors (may need to change depending on orientation of robot)
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);




        // Reset the encoder count - intialize to 0
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry to the robot
        telemetry.addLine("Working");
        // telemetry.addData("Last updated",sdf.format(file.lastModified()));

        //imu learning attempt
        //turning variables (M# = move #)

        float currentAngle, panningAngle;
        float finalRotAngleM2 = -84.18f;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


    }

    @Override
    public void init_loop ()
    {
        // pls do
    }

    @Override
    public void loop ()
    {
        //flags
        boolean move1Finished = false;
        boolean move2Finished = false;
        boolean move3Finished = false;

        boolean finishedTurning = false;






        frontLeftMotor.setPower(0.3);
        frontRightMotor.setPower(0.3);
        backLeftMotor.setPower(0.3);
        backRightMotor.setPower(0.3);

        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
        currentAngle = lastAngles.firstAngle - zeroAngle;


        liftMotor.setPower(0.4);
        currentAngle = lastAngles.firstAngle - zeroAngle;
        telemetry.addData("Current Y angle",lastAngles.firstAngle);
        telemetry.addData("Current X angle",lastAngles.secondAngle);
        telemetry.addData("Current Z angle",lastAngles.thirdAngle);
        telemetry.addData("zero angle",zeroAngle);

        /*
        //move1: Forward

        if (!move1Finished && !move2Finished && !move3Finished)
        {
            frontLeftMotor.setTargetPosition(2000);
            frontRightMotor.setTargetPosition(2000);
            backLeftMotor.setTargetPosition(2000);
            backRightMotor.setTargetPosition(2000);

            //test number, do calculations
            liftMotor.setTargetPosition(3500);

            rotateServo.setPower(0.5);
            clawServo.setPower(0.52);

            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);





            //check move 1
            if (Math.abs(frontLeftMotor.getCurrentPosition() - 2000) < 5 && Math.abs(frontRightMotor.getCurrentPosition() - 2000) < 5 &&
                    Math.abs(backLeftMotor.getCurrentPosition() - 2000) < 5 && Math.abs(backRightMotor.getCurrentPosition() - 2000) < 5)
            {
                move1Finished = true;
                frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            }
        }

        //move2: turn left 90 degrees
        if (move1Finished && !move2Finished && !move3Finished)
        {
            telemetry.addLine("First move completed successfully!");


            lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
            float zeroAngle = lastAngles.firstAngle;
            currentAngle = lastAngles.firstAngle - zeroAngle;

            finalRotAngleM2 = 76.25f;

            if (currentAngle < finalRotAngleM2)
            {

                currentAngle = lastAngles.firstAngle - zeroAngle;
                telemetry.addData("Current angle",currentAngle);
                frontLeftMotor.setTargetPosition(-2595);
                frontRightMotor.setTargetPosition(2595);
                backLeftMotor.setTargetPosition(-2559);
                backRightMotor.setTargetPosition(2559);

                frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
            if (currentAngle > finalRotAngleM2)
            {
                move2Finished = true;
                frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            }
        }

        // move3: dunk and drop cone
        if (move1Finished && move2Finished && !move3Finished)
        {
            telemetry.addLine("Second move completed successfully!");

            rotateServo.setPower(-0.7);
            clawServo.setPower(0.1);

            move3Finished = true;



        }
        if (move1Finished && move2Finished && move3Finished)
        {
            telemetry.addLine("Task failed successfully!");
        }
*/


    }
}
