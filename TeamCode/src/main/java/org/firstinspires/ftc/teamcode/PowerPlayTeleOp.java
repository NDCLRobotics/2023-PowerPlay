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
    private CRServo clawServo = null;
    private CRServo rotateServo = null;

    private int liftMotorPos;
    private int liftMotorZero;

    double clawPos = 0.4;
    double rotatePos = -0.1;

    // Frames
    private long currentFrame;
    private long startHomeFrame;

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

        clawServo = hardwareMap.crservo.get("clawServo");
        rotateServo = hardwareMap.crservo.get("rotateServo");

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

    private void autoHoming()
    {
        long retractTime = startHomeFrame + 100; // How long it takes to retract arm
        long centerTime = retractTime + 200; // How long it takes to center and lower arm


        clawPos = 0.1;
        rotatePos = -0.1;

        // Lowers the lift motor until, it reaches the zero position
        if (liftMotorPos > liftMotorZero)
        {
            liftMotor.setPower(-0.6);
        }
        else
        {
            liftMotor.setPower(0.0);
        }

        if (currentFrame > startHomeFrame && currentFrame <= retractTime)
        {
            extendServo.setPower(extendPos);
            telemetry.addData("Retracting","");
        }
        if (currentFrame > retractTime && currentFrame <= centerTime) // raising
        {
            spinServo.setPower(spinPos);
            //liftMotor.setTargetPosition(liftMotorZero);
            telemetry.addData("Centering and Lowering","");
        }
        if (currentFrame > centerTime)
        {
            // stop
            autoHome = false;
        }
    }

    @Override
    public void loop ()
    {
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double pan = -gamepad1.left_stick_x;

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


        if (liftMotorPos < liftMotorZero + 5750 && gamepad1.right_trigger > 0)
        {
            liftMotor.setPower(1.0);
        }
        else if (liftMotorPos > liftMotorZero + 5750 && gamepad1.right_trigger > 0)
        {
            liftMotor.setPower(0.6);
        }
        else if (liftMotorPos > liftMotorZero + 850 && gamepad1.left_trigger > 0)
        {
            liftMotor.setPower(-0.8);
        }
        else if (liftMotorPos < liftMotorZero + 850 && gamepad1.left_trigger > 0)
        {
            liftMotor.setPower(-0.2);
        }
        else
        {
            liftMotor.setPower(0.0);
        }

        if (liftMotorPos < liftMotorZero)
        {
            liftMotor.setPower(0.1);
        }
        if (liftMotorPos > liftMotorZero + 6500)
        {
            liftMotor.setPower(-0.2);
        }


        if (gamepad1.right_bumper) // Open position for claw
        {
            clawServo.setPower(0.1);
        }
        else if (gamepad1.left_bumper) // Closed position for claw
        {
            clawServo.setPower(0.48);
        }

        if (gamepad1.triangle) // Up position for claw
        {
            rotateServo.setPower(0.5);
        }
        if (gamepad1.cross) // Down for cargo grip
        {
            rotateServo.setPower(-0.7);
        }
        else if (gamepad1.square) // horizontal for cargo grip
        {
            rotateServo.setPower(rotatePos);
        }


        // Clamp for driving power scale
        powerScale = Range.clip(powerScale, 0.2, 1.0);

        // Output telemetry
        telemetry.addLine("Power Scale:" + powerScale);
        telemetry.addLine("Lift Position:" + liftMotorPos);
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