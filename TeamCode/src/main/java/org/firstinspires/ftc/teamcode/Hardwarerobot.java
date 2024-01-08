package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.Encoder;
@Config
public class Hardwarerobot
{
    public DcMotor  leftFrontDrive   = null;
    public DcMotor  leftBackDrive   = null;
    public DcMotor  rightFrontDrive  = null;
    public DcMotor  rightBackDrive  = null;
    public DcMotor viperSlideLift = null;
    public DcMotor intake = null;
    public DcMotor slideMotor = null;


    public Servo viperServo1 = null;
    public Servo viperServo2 = null;
    public Servo pixelBox = null;
    public Servo pixelHolderRotator = null;
    public Servo pixelHolderDoor = null;
    public Servo intakeBox = null;
    public Servo droneLauncher = null;
    public Servo auto = null;


    public static double PIXELHOLDERROTATOR_STORE_POS = 0.75; //.65
    public static double PIXELHOLDERROTATOR_DEPOSIT_POS = 0;

    public static double AUTO_CLOSED_POS = 0.5;
    public static double AUTO_OPEN_POS = 0;

    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    public Hardwarerobot(){
    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        leftFrontDrive  = hwMap.get(DcMotor.class, "FL");
        leftBackDrive  = hwMap.get(DcMotor.class, "BL");
        rightFrontDrive = hwMap.get(DcMotor.class, "FR");
        rightBackDrive = hwMap.get(DcMotor.class, "BR");
        viperSlideLift = hwMap.get(DcMotor.class, "viperSlideLift");
        intake =  hwMap.get(DcMotor.class, "intake");
        slideMotor  = hwMap.get(DcMotor.class, "slideMotor");
        viperServo1 = hwMap.get(Servo.class,"viperServo1");
        viperServo2 = hwMap.get(Servo.class, "viperServo2");
        pixelBox = hwMap.get(Servo.class, "pixelBox");
        pixelHolderRotator = hwMap.get(Servo.class,"pixelHolderRotator");
        pixelHolderDoor = hwMap.get(Servo.class,"pixelHolderDoor");
        intakeBox = hwMap.get(Servo.class,"intakeBox");
        droneLauncher = hwMap.get(Servo.class,"droneLauncher");
        auto = hwMap.get(Servo.class, "auto_servo");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        slideMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        viperSlideLift.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);

        viperServo1.setDirection(Servo.Direction.FORWARD);
        viperServo2.setDirection(Servo.Direction.REVERSE);
        pixelBox.setDirection(Servo.Direction.FORWARD);
        pixelHolderRotator.setDirection(Servo.Direction.REVERSE);
        pixelHolderDoor.setDirection(Servo.Direction.REVERSE);
        intakeBox.setDirection(Servo.Direction.REVERSE);
        droneLauncher.setDirection(Servo.Direction.FORWARD);
        auto.setDirection(Servo.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperSlideLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        intake.setPower(0);
        viperSlideLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlideLift.setTargetPosition(0);
        viperSlideLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideLift.setPower(0);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setTargetPosition(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(0);

        leftBackDrive.setTargetPosition(0);
        slideMotor.setTargetPosition(0);
        leftFrontDrive.setTargetPosition(0);
        rightBackDrive.setTargetPosition(0);
        rightFrontDrive.setTargetPosition(0);
        intake.setTargetPosition(0);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        viperServo1.setPosition(0);
        viperServo2.setPosition(0);
        pixelBox.setPosition(0);
        pixelHolderRotator.setPosition(0.65);
        pixelHolderDoor.setPosition(1);
        intakeBox.setPosition(.5);
        droneLauncher.setPosition(0);
        auto.setPosition(AUTO_CLOSED_POS);
    }
}
