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

@TeleOp(name="Power Play TeleOp", group="Interactive Opmode")

public class PowerPlayTeleOp extends OpMode
{
    // Control Hub
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor liftMotor = null;

    private int liftMotorPos;
    private int liftMotorZero;

    // Variables for power set to the drive and pan functions
    private double frontLeftPower, frontRightPower, backLeftPower, backRightPower;
    private double frontLeftPan, frontRightPan, backLeftPan, backRightPan;

    // Scale variable
    private double powerScale = 0.8;
    private boolean powerSwitching = false;

    // Pull the date of the file (to update the date of telemetry)
    File curDir = new File("./system");
    private static Stack<File> nest = new Stack<File>();

    public void getAllFiles (File curDir)
    {
        SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd HH:mm");
        File[] filesList = curDir.listFiles();
        if (filesList != null)
        {
            for (File f : filesList)
            {
                if (f.isDirectory() && !f.getName().equals("bin"))// && nest.size() < 3)
                {
                    nest.push(f);
                    getAllFiles(f);
                    nest.pop();
                }
                else //if (f.isFile())
                {
                    if (sdf.format(f.lastModified()).contains("2022"))
                    {
                        String s = "";
                        for (File ff : nest)
                        {
                            s += ff.getName() + "/";
                        }
                        if (f.isDirectory())
                        {
                            telemetry.addLine(s + f.getName() + "/");
                        }
                        else
                        {
                            telemetry.addLine(s + f.getName() + " " + sdf.format(f.lastModified()));
                        }
                    }
                }
            }
        }
    }

    @Override
    public void init ()
    {
        // Initialize connection to motors
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        liftMotor = hardwareMap.dcMotor.get("liftMotor");

        // Set direction to the motors (may need to change depending on orientation of robot)
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        
        liftMotorZero = liftMotor.getCurrentPosition();

        // Send telemetry to the robot
        telemetry.addLine("Working");
        // telemetry.addData("Last updated",sdf.format(file.lastModified()));

        // test stuff
        getAllFiles(curDir);
    }

    @Override
    public void init_loop ()
    {
        // pls do
    }

    @Override
    public void loop ()
    {
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double pan = gamepad1.left_stick_x;

        liftMotorPos = liftMotor.getCurrentPosition();

        // Driving controls
        frontLeftPower = Range.clip(drive + turn, -1.0, 1.0);
        frontRightPower = Range.clip(drive - turn, -1.0, 1.0);
        backLeftPower = Range.clip(drive + turn, -1.0, 1.0);
        backRightPower = Range.clip(drive - turn, -1.0, 1.0);
        frontLeftMotor.setPower(powerScale * frontLeftPower);
        frontRightMotor.setPower(powerScale * frontRightPower);
        backLeftMotor.setPower(powerScale * backLeftPower);
        backRightMotor.setPower(powerScale * backRightPower);

        // Panning controls
        frontLeftPan = Range.clip(drive + pan, -1.0, 1.0);
        frontRightPan = Range.clip(drive - pan, -1.0, 1.0);
        backLeftPan = Range.clip(drive - pan, -1.0, 1.0);
        backRightPan = Range.clip(drive + pan, -1.0, 1.0);
        frontLeftMotor.setPower(powerScale * frontLeftPan);
        frontRightMotor.setPower(powerScale * frontRightPan);
        backLeftMotor.setPower(powerScale * backLeftPan);
        backRightMotor.setPower(powerScale * backRightPan);

        // Incrementing speed for driving motor, up speeds up motors, down slows down motors
        if (gamepad1.dpad_up && !powerSwitching)
        {
            powerSwitching = true;
            powerScale += 0.2;
        }
        if (gamepad1.dpad_down && !powerSwitching)
        {
            powerSwitching = true;
            powerScale -= 0.2;
        }
        if (!gamepad1.dpad_down && !gamepad1.dpad_up && powerSwitching)
        {
            powerSwitching = false;
        }
        if (gamepad1.right_trigger > 0)
        {
            liftMotor.setPower(1.0);
        }
        else if (gamepad1.left_trigger > 0 )
        {
            liftMotor.setPower(-1.0);
    }
        else
        {
            liftMotor.setPower(0.0);
        }
        // Clamp for driving power scale
        powerScale = Range.clip(powerScale, 0.2, 1.0);

        // Output telemetry
        telemetry.addData("Power Scale:", powerScale);
    }

    @Override
    public void stop ()
    {
        // I don't know how many years in a row I have to reiterate this, pls do
        frontLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
        liftMotor.setPower(0.0);
    }
}