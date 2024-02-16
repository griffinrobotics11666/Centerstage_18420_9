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
    public DcMotor viperSlideLifttwo = null;
    public DcMotor intake = null;
    public Servo pixelBox = null;
    public Servo pixelHolderRotator = null;
    public Servo droneLauncher = null;
    public Servo auto = null;
    public Servo pinchertheright = null;
    public Servo pinchertheleft = null;
    public Servo wrist = null;
    public Servo mosaic = null;


    public static double PIXELHOLDERROTATOR_STORE_POS = 0; //.75
    public static double PIXELHOLDERROTATOR_INIT_POS = 0.025;
    public static double PIXELHOLDERROTATOR_DEPOSIT_POS = 0.7;
    public static double PIXELHOLDERROTATOR_AUTO_POS = .87;
    public static double PIXELHOLDERROTATOR_REDCENTER_POS = .88;

    public static double AUTO_CLOSED_POS = 0.5;
    public static double AUTO_OPEN_POS = 0;

    public static double WRIST_DEPOSIT_POS = 0.15;
    public static double WRIST_STORE_POS = 0.0;
    public static double WRIST_REDCENTER_POS = .18;

    public static double MOSAIC_DEPOSIT_POS = .2;
    public static double MOSAIC_STORE_POS = 0;

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
        viperSlideLifttwo = hwMap.get(DcMotor.class, "viperSlideLifttwo");
        intake =  hwMap.get(DcMotor.class, "intake");
        pixelBox = hwMap.get(Servo.class, "pixelBox");
        pixelHolderRotator = hwMap.get(Servo.class,"pixelHolderRotator");
        droneLauncher = hwMap.get(Servo.class,"droneLauncher");
        auto = hwMap.get(Servo.class, "auto_servo");
        pinchertheright = hwMap.get(Servo.class, "pinchertheright");
        pinchertheleft = hwMap.get(Servo.class, "pinchertheleft");
        wrist = hwMap.get(Servo.class, "wrist");
        mosaic = hwMap.get(Servo.class, "mosaic");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        viperSlideLift.setDirection(DcMotor.Direction.FORWARD);
        viperSlideLifttwo.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);

        pixelBox.setDirection(Servo.Direction.REVERSE);
        pixelHolderRotator.setDirection(Servo.Direction.REVERSE);
        droneLauncher.setDirection(Servo.Direction.FORWARD);
        auto.setDirection(Servo.Direction.FORWARD);
        pinchertheright.setDirection(Servo.Direction.REVERSE);
        pinchertheleft.setDirection(Servo.Direction.FORWARD);
        wrist.setDirection(Servo.Direction.REVERSE);
        mosaic.setDirection(Servo.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperSlideLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperSlideLifttwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        intake.setPower(0);
        viperSlideLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlideLift.setTargetPosition(0);
        viperSlideLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideLift.setPower(0);
        viperSlideLifttwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlideLifttwo.setTargetPosition(0);
        viperSlideLifttwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideLifttwo.setPower(0);

        leftBackDrive.setTargetPosition(0);
        leftFrontDrive.setTargetPosition(0);
        rightBackDrive.setTargetPosition(0);
        rightFrontDrive.setTargetPosition(0);
        intake.setTargetPosition(0);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pixelBox.setPosition(0);
        pixelHolderRotator.setPosition(PIXELHOLDERROTATOR_INIT_POS);
        droneLauncher.setPosition(0);
        auto.setPosition(AUTO_CLOSED_POS);
        pinchertheleft.setPosition(DriverControl2024.PINCHERLEFTOPEN);
        pinchertheright.setPosition(DriverControl2024.PINCHERRIGHTOPEN);
        wrist.setPosition(0);
        mosaic.setPosition(0);
    }
}
