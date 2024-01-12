package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@Config
@TeleOp(name="Driver control 2024", group="TeleOp")
public class DriverControl2024 extends OpMode {
    private ElapsedTime runtime = new ElapsedTime(); //clock
    private ElapsedTime liftTimer = new ElapsedTime();
    private ElapsedTime slideTimer = new ElapsedTime();
    private ElapsedTime droneSaftey = new ElapsedTime();
    private ElapsedTime hangSaftey = new ElapsedTime();
    double intakeDownDelayTime = 800;
    double intakeUpDelayTime = 800;
    double droneSafteyTimer = 90000;
    double hangSafteyTimer = 0;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    public enum LiftState {
        LIFT_UP,
        LIFT_LOWER
    }
    LiftState liftState = LiftState.LIFT_LOWER;
    public enum SlideState {
        SLIDE_READY,
        SLIDE_EXTEND,
        SLIDE_IDLE,
        SLIDE_RETRACT
    }
    SlideState slideState = SlideState.SLIDE_READY;

    public static double PIXELHOLDERDOOR_STORE_POS = 0.9;
    public static double PIXELHOLDERDOOR_DEPOSIT_POS = 0.5;
    public static double PIXELDOOR2_STORE_POS = 0.9;
    public static double PIXELDOOR2_DEPOSIT_POS = 0.5;
    public static double INTAKEBOXUP = .68;
    public static double INTAKEBOXINIT = .5;
    public static double INTAKEBOXDOWN = 0.03;
    public static double CLOSEPOSITION = 0;
    public static double FIRE = .5;
    public static double HANGCLOSEPOS = 0;
    public static double HANGHANGINGPOS = 0.5;
    //public static double SERVODESPOSITEPOS = 0.5;
    //public static double SERVOSTOREPOS = 0;

    boolean lastMovement= false; boolean currentMovement= false;
    boolean downPosition= true;

    boolean lastMovement2= false; boolean currentMovement2= false;
    boolean downPosition2= true;

    boolean lastMovement3 = false; boolean currentMovement3 = false;
    boolean downPosition3 = true;

    boolean lastMovement4 = false; boolean currentMovement4 = false;
    boolean downPosition4 = true;

    boolean lastMovement5 = false; boolean currentMovement5 = false;
    boolean downPosition5 = true;

    Hardwarerobot robot = new Hardwarerobot();
    HardwareMap hwMap = null;
    double slowfactor = 0.5;

    int newTarget = 0;

    static double ARM_COUNTS_PER_INCH = 80; //Figure out right number //114.75
    static double SLIDE_COUNTS_PER_INCH = 80;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);
        telemetry.addData("Status", "Ready");

        robot.viperSlideLift.setDirection(DcMotor.Direction.REVERSE);
        robot.viperSlideLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.viperSlideLift.setTargetPosition(0);
        robot.viperSlideLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.viperSlideLift.setPower(1);

        robot.slideMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slideMotor.setTargetPosition(0);
        robot.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideMotor.setPower(.7);

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        double leftFrontPower;
        double leftBackPower;
        double rightFrontPower;
        double rightBackPower;

        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_x;

        leftFrontPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
        leftBackPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
        rightFrontPower = Range.clip(drive - turn - strafe, -1.0, 1.0);
        rightBackPower = Range.clip(drive - turn + strafe, -1.0, 1.0);

        if (gamepad1.right_trigger > 0) {
            leftFrontPower = leftFrontPower * slowfactor;
            leftBackPower = leftBackPower * slowfactor;
            rightFrontPower = rightFrontPower * slowfactor;
            rightBackPower = rightBackPower * slowfactor;
        }

        robot.leftFrontDrive.setPower(leftFrontPower);
        robot.leftBackDrive.setPower(leftBackPower);
        robot.rightFrontDrive.setPower(rightFrontPower);
        robot.rightBackDrive.setPower(rightBackPower);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower);
        telemetry.addData("Status", "BRENNAN AND EGG MAN YOU BETTER NOT GET ANY PENALTIES OR ELSE . . .");
        telemetry.update();

        if (Math.abs(gamepad2.left_trigger - gamepad2.right_trigger) > .1) {
            robot.intake.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
        }
        else{
            robot.intake.setPower(0);
        }

        if (droneSaftey.milliseconds() >= droneSafteyTimer) {
            if (gamepad1.a) {
                robot.droneLauncher.setPosition(FIRE);
            } else {
                robot.droneLauncher.setPosition(CLOSEPOSITION);
            }
        }

        lastMovement = currentMovement;
        currentMovement = gamepad2.y;

        if (currentMovement && !lastMovement) {
            downPosition = !downPosition;
            if (downPosition) {
                robot.pixelHolderRotator.setPosition(robot.PIXELHOLDERROTATOR_DEPOSIT_POS);
            } else {
                robot.pixelHolderRotator.setPosition(robot.PIXELHOLDERROTATOR_STORE_POS);
            }
        }
/*
        lastMovement2 = currentMovement2;
        currentMovement2 = gamepad2.b;

        if (currentMovement2 && !lastMovement2) {
            downPosition2 = !downPosition2;
            if (downPosition2) {
                robot.pixelHolderDoor.setPosition(PIXELHOLDERDOOR_DEPOSIT_POS); //Move Down
            } else {
                robot.pixelHolderDoor.setPosition(PIXELHOLDERDOOR_STORE_POS);
            }
        }

 */

        lastMovement3 = currentMovement3;
        currentMovement3 = gamepad2.a;

        if (currentMovement3 && !lastMovement3) {
            downPosition3 = !downPosition3;
            if (downPosition3) {
                robot.intakeBox.setPosition(INTAKEBOXDOWN);
            } else {
                robot.intakeBox.setPosition(INTAKEBOXUP);
            }
        }

        lastMovement4 = currentMovement4;
        currentMovement4 = gamepad2.dpad_right;

        if (hangSaftey.milliseconds() >= hangSafteyTimer) {
            if (currentMovement4 && !lastMovement4) {
                downPosition4 = !downPosition4;
                if (downPosition4) {
                    hang();
                } else {

                }
            }
        }

        lastMovement5 = currentMovement5;
        currentMovement5 = gamepad2.b;

        if (currentMovement5 && !lastMovement5) {
            downPosition5 = !downPosition5;
            if (downPosition5) {
                robot.auto.setPosition(robot.AUTO_CLOSED_POS);
            } else {
                robot.auto.setPosition(robot.AUTO_OPEN_POS);
            }
        }
        telemetry.addData("position",robot.pixelHolderRotator.getPosition());
        telemetry.update();


        if (gamepad2.dpad_up){
            goTo3();
        }

        switch (liftState) {
            case LIFT_UP:
                if (gamepad2.dpad_down){
                    robot.pixelHolderDoor.setPosition(PIXELHOLDERDOOR_DEPOSIT_POS);
                    robot.pixelHolderRotator.setPosition(robot.PIXELHOLDERROTATOR_STORE_POS);
                    goTo0();
                    liftState = LiftState.LIFT_LOWER;
                    liftTimer.reset();
                }
                break;

            case LIFT_LOWER:
                if(gamepad2.dpad_up){
                    robot.pixelHolderDoor.setPosition(PIXELHOLDERDOOR_STORE_POS);
                    goTo3();
                    robot.pixelHolderRotator.setPosition(robot.PIXELHOLDERROTATOR_DEPOSIT_POS);
                    liftState = LiftState.LIFT_UP;
                    liftTimer.reset();
                }
                break;
        }


        switch (slideState) {
            case SLIDE_READY:
                if (gamepad2.right_bumper) {
                    robot.intakeBox.setPosition(INTAKEBOXDOWN);
                    slideState = SlideState.SLIDE_EXTEND;
                    slideTimer.reset();
                }
                break;
            case SLIDE_EXTEND:
                if (slideTimer.milliseconds() >= intakeDownDelayTime) {
                    extend();
                    slideState = SlideState.SLIDE_IDLE;
                    slideTimer.reset();
                }
                break;
            case SLIDE_IDLE:
                if (gamepad2.left_bumper) {
                robot.intakeBox.setPosition(INTAKEBOXINIT);
                slideState = SlideState.SLIDE_RETRACT;
                slideTimer.reset();
            }
                break;
            case SLIDE_RETRACT:
                if (slideTimer.milliseconds() >= intakeUpDelayTime) {
                    retract();
                    slideState = SlideState.SLIDE_READY;
                    slideTimer.reset();
                }
                break;
        }

    }

    public void goTo0() {
        double distance = 0;
        newTarget = (int) (distance *ARM_COUNTS_PER_INCH);
        robot.viperSlideLift.setTargetPosition(newTarget);

    }
    public void goTo3(){
        double distance = 15;
        newTarget = (int) (distance *ARM_COUNTS_PER_INCH);
        robot.viperSlideLift.setTargetPosition(newTarget);
    }
    public void extend() {
        double distance = 4;
        newTarget = (int) (distance * SLIDE_COUNTS_PER_INCH);
        robot.slideMotor.setTargetPosition(newTarget);
    }
    public void retract() {
        double distance = 0;
        newTarget = (int) (distance * SLIDE_COUNTS_PER_INCH);
        robot.slideMotor.setTargetPosition(newTarget);
    }
    public void hang() {
        double distance = 26;
        newTarget = (int) (distance * ARM_COUNTS_PER_INCH);
        robot.viperSlideLift.setTargetPosition(newTarget);
    }

    @Override
    public void stop() {
    }
}