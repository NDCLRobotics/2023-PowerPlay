package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import java.io.File;
import java.text.SimpleDateFormat;
import java.util.Stack;
import java.lang.Math;

@TeleOp(name="Second Robot Encoder Test", group="Interactive Opmode")

public class SecondRobotEncoderTest extends OpMode
{


    //flags
    private boolean run_auto = false;
    private boolean move1finished=false;
    private boolean move2finished=false;
    private boolean move3finished=false;
    private boolean move4finished=false;


    // Control Hub
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;

    @Override
    public void init ()
    {
        // Initialize connection to motors
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        // Set direction to the motors (may need to change depending on orientation of robot)
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset the encoder count - intialize to 0
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
        // Reset encoder positions
        if (gamepad1.cross)
        {
            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if (gamepad1.circle) {
            run_auto = true;
        }

        if (run_auto)
        {
            if (!move1finished)
            {
                //1.move left
                frontLeftMotor.setTargetPosition(759);
                frontRightMotor.setTargetPosition(759);
                backLeftMotor.setTargetPosition(-759);
                backRightMotor.setTargetPosition(-759);

                frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                frontLeftMotor.setPower(0.4);
                frontRightMotor.setPower(0.4);
                backLeftMotor.setPower(0.4);
                backRightMotor.setPower(0.4);

                // switch to 2nd move when all encoders are within 5 of target value
                if (Math.abs(frontLeftMotor.getCurrentPosition() - 759) < 5 && Math.abs(frontRightMotor.getCurrentPosition() - 759) < 5 &&
                        Math.abs(backLeftMotor.getCurrentPosition() + 759) < 5 && Math.abs(backRightMotor.getCurrentPosition() + 759) < 5)
                {
                    move1finished = true;
                    frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                }
            }

            if (move1finished && !move2finished)
            {
                // 2.move forward
                telemetry.addLine("Starting 2nd move");
                frontLeftMotor.setTargetPosition(-759);
                frontRightMotor.setTargetPosition(759);
                backLeftMotor.setTargetPosition(-759);
                backRightMotor.setTargetPosition(759);

                frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                frontLeftMotor.setPower(0.4);
                frontRightMotor.setPower(0.4);
                backLeftMotor.setPower(0.4);
                backRightMotor.setPower(0.4);

                if (Math.abs(frontLeftMotor.getCurrentPosition() + 759) < 5 && Math.abs(frontRightMotor.getCurrentPosition() - 759) < 5 &&
                        Math.abs(backLeftMotor.getCurrentPosition() + 759) < 5 && Math.abs(backRightMotor.getCurrentPosition() - 759) < 5)
                {
                    move2finished = true;
                    frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }

            }
            if (move2finished && !move3finished)
            {
                // 3.move right
                telemetry.addLine("Starting 3rd move");
                frontLeftMotor.setTargetPosition(-759);
                frontRightMotor.setTargetPosition(-759);
                backLeftMotor.setTargetPosition(759);
                backRightMotor.setTargetPosition(759);

                frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                frontLeftMotor.setPower(0.4);
                frontRightMotor.setPower(0.4);
                backLeftMotor.setPower(0.4);
                backRightMotor.setPower(0.4);

                if (Math.abs(frontLeftMotor.getCurrentPosition() + 759) < 5 && Math.abs(frontRightMotor.getCurrentPosition() + 759) < 5 &&
                        Math.abs(backLeftMotor.getCurrentPosition() - 759) < 5 && Math.abs(backRightMotor.getCurrentPosition() - 759) < 5)
                {
                    move3finished = true;
                    frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }

            }
            if (move3finished && !move4finished)
            {
                // 4.move back
                telemetry.addLine("Starting 4th move");
                frontLeftMotor.setTargetPosition(759);
                frontRightMotor.setTargetPosition(-759);
                backLeftMotor.setTargetPosition(759);
                backRightMotor.setTargetPosition(-759);

                frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                frontLeftMotor.setPower(0.4);
                frontRightMotor.setPower(0.4);
                backLeftMotor.setPower(0.4);
                backRightMotor.setPower(0.4);

                if (Math.abs(frontLeftMotor.getCurrentPosition() - 759) < 5 && Math.abs(frontRightMotor.getCurrentPosition() + 759) < 5 &&
                        Math.abs(backLeftMotor.getCurrentPosition() - 759) < 5 && Math.abs(backRightMotor.getCurrentPosition() + 759) < 5)
                {
                    move4finished = true;
                    frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }

            }


            // reset flags
            if (move4finished)
            {
                move1finished = false;
                move2finished = false;
                move3finished = false;
                move4finished = false;
                run_auto = false;
            }





        }



        // Output telemetry
        telemetry.addData("Front Left", frontLeftMotor.getCurrentPosition());
        telemetry.addData("FL Target", frontLeftMotor.getTargetPosition());

        telemetry.addData("Front Right", frontRightMotor.getCurrentPosition());
        telemetry.addData("FR Target", frontRightMotor.getTargetPosition());

        telemetry.addData("Back Left", backLeftMotor.getCurrentPosition());
        telemetry.addData("BL Target", backLeftMotor.getTargetPosition());

        telemetry.addData("Back Right", backRightMotor.getCurrentPosition());
        telemetry.addData("BR Target", backRightMotor.getTargetPosition());

        telemetry.addData("Move 1 finished?", move1finished);
        telemetry.addData("Move 2 finished?", move2finished);
        telemetry.addData("Move 3 finished?", move3finished);
        telemetry.addData("Move 4 finished?", move4finished);
    }

    @Override
    public void stop ()
    {
        // I don't know how many years in a row I have to reiterate this, pls do
        frontLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
    }
}