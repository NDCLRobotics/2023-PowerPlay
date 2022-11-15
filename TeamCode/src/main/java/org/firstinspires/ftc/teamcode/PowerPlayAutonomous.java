package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import java.io.File;
import java.text.SimpleDateFormat;
import java.util.Stack;

@TeleOp(name="Power Play Auto", group="Interactive Opmode")
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
    }

    @Override
    public void init_loop ()
    {
        // pls do
    }

    @Override
    public void loop ()
    {
        boolean move1Finished = false;
        boolean move2Finished = false;
        boolean move3Finished = false;


        frontLeftMotor.setPower(0.3);
        frontRightMotor.setPower(0.3);
        backLeftMotor.setPower(0.3);
        backRightMotor.setPower(0.3);

        liftMotor.setPower(0.2);

        //move1: Forward

        if (!move1Finished && !move2Finished && !move3Finished)
        {
            frontLeftMotor.setTargetPosition(2595);
            frontRightMotor.setTargetPosition(2595);
            backLeftMotor.setTargetPosition(2559);
            backRightMotor.setTargetPosition(2559);

            //test number, do calculations
            liftMotor.setTargetPosition(50);

            rotateServo.setPower(0.5);

            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);





            //check move 1
            if (Math.abs(frontLeftMotor.getCurrentPosition() - 2559) < 5 && Math.abs(frontRightMotor.getCurrentPosition() - 2559) < 5 &&
                    Math.abs(backLeftMotor.getCurrentPosition() - 2559) < 5 && Math.abs(backRightMotor.getCurrentPosition() - 2559) < 5)
            {
                move1Finished = true;
                frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            }
        }


        if (move1Finished && !move2Finished && !move3Finished)
        {
            telemetry.addLine("First move completed successfully!");


            frontLeftMotor.setTargetPosition(2595);
            frontRightMotor.setTargetPosition(2595);
            backLeftMotor.setTargetPosition(2559);
            backRightMotor.setTargetPosition(2559);




        }





    }
}
