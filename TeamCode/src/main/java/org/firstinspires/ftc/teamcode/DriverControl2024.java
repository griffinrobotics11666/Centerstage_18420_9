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

    private ElapsedTime droneSaftey = new ElapsedTime();
    double droneSafteyTimer = 90000;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    public enum LiftState {
        LIFT_UP,
        LIFT_LOWER,
        LIFT_HANG;
    }
    LiftState liftState = LiftState.LIFT_LOWER;

    public static double CLOSEPOSITION = 0;
    public static double FIRE = .5;
    public static double PINCHERLEFTOPEN = 0.1;
    public static double PINCHERLEFTCLOSE = 0.5;
    public static double PINCHERRIGHTOPEN = 0.1;
    public static double PINCHERRIGHTCLOSE = 0.5;
    public static double outTakeClosed = 0;
    public static double outTakeMiddle = 0.075;
    public static double getOutTakeOpen = .15;

    boolean lastMovement1 = false; boolean currentMovement1 = false;
    boolean downPosition1 = true;

    boolean lastMovement2 = false; boolean currentMovement2 = false;
    boolean downPosition2 = true;

    boolean lastMovement3 = false; boolean currentMovement3 = false;
    boolean downPosition3 = true;

    boolean lastMovement4 = false; boolean currentMovement4 = false;
    boolean downPosition4 = true;

    Hardwarerobot robot = new Hardwarerobot();
    HardwareMap hwMap = null;
    double slowfactor = 0.5;

    int newTarget = 0;

    static double ARM_COUNTS_PER_INCH = 80;

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

        robot.viperSlideLifttwo.setDirection(DcMotor.Direction.FORWARD);
        robot.viperSlideLifttwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.viperSlideLifttwo.setTargetPosition(0);
        robot.viperSlideLifttwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.viperSlideLifttwo.setPower(1);

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
        double turn = -gamepad1.right_stick_x;
        double strafe = -gamepad1.left_stick_x;

        leftFrontPower = Range.clip(drive - turn - strafe, -1.0, 1.0);
        leftBackPower = Range.clip(drive - turn + strafe, -1.0, 1.0);
        rightBackPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
        rightFrontPower = Range.clip(drive + turn + strafe, -1.0, 1.0);

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


        if(gamepad2.left_bumper == true){
         robot.pixelBox.setPosition(getOutTakeOpen);
        } else if(gamepad2.right_bumper == true){
            robot.pixelBox.setPosition(outTakeMiddle);
        } else {
            robot.pixelBox.setPosition(outTakeClosed);
        }






        lastMovement1 = currentMovement1;
        currentMovement1 = gamepad2.y;

        if (currentMovement1 && !lastMovement1) {
            downPosition1 = !downPosition1;
            if (downPosition1) {
                robot.wrist.setPosition(robot.WRIST_STORE_POS);
                robot.pixelHolderRotator.setPosition(robot.PIXELHOLDERROTATOR_DEPOSIT_POS);
            } else {
                robot.wrist.setPosition(robot.WRIST_STORE_POS);
                robot.pixelHolderRotator.setPosition(robot.PIXELHOLDERROTATOR_STORE_POS);
            }
        }

        lastMovement2 = currentMovement2;
        currentMovement2 = gamepad1.b;

        if (currentMovement2 && !lastMovement2) {
            downPosition2 = !downPosition2;
            if(downPosition2){
                robot.mosaic.setPosition(robot.MOSAIC_DEPOSIT_POS);
            } else {
                robot.mosaic.setPosition(robot.MOSAIC_STORE_POS);
            }
        }

        lastMovement3 = currentMovement3;
        currentMovement3 = gamepad2.a;

        if (currentMovement3 && !lastMovement3) {
            downPosition3 = !downPosition3;
            if (downPosition3) {
                robot.pixelHolderRotator.setPosition(robot.PIXELHOLDERROTATOR_AUTO_POS);
                robot.wrist.setPosition(robot.WRIST_DEPOSIT_POS);
            } else {
                robot.pixelHolderRotator.setPosition(robot.PIXELHOLDERROTATOR_AUTO_POS);
                robot.wrist.setPosition(robot.WRIST_STORE_POS);
            }
        }


        if (gamepad2.b) {
            robot.pinchertheright.setPosition(PINCHERRIGHTCLOSE);
            robot.pinchertheleft.setPosition(PINCHERLEFTCLOSE);
        }else{
            robot.pinchertheright.setPosition(PINCHERRIGHTOPEN);
            robot.pinchertheleft.setPosition(PINCHERLEFTOPEN);
        }

        if (Math.abs(gamepad2.left_stick_y)>.1){
            double distance = -5*gamepad2.left_stick_y;
            newTarget = robot.viperSlideLift.getCurrentPosition()+ (int) (distance * ARM_COUNTS_PER_INCH);
            robot.viperSlideLift.setTargetPosition(newTarget);
            robot.viperSlideLifttwo.setTargetPosition(newTarget);
            liftState = LiftState.LIFT_UP;
        }

        switch (liftState) {

            case LIFT_LOWER:
                if (gamepad2.dpad_up){
                    goTo1();
                    robot.pixelHolderRotator.setPosition(robot.PIXELHOLDERROTATOR_DEPOSIT_POS);
                    liftState = LiftState.LIFT_UP;
                    liftTimer.reset();
                }

            case LIFT_UP:
                if (gamepad2.dpad_right){
                    hang();
                    liftState = LiftState.LIFT_HANG;
                    liftTimer.reset();
                }
                if (gamepad2.dpad_down){
                    goTo0();
                    liftState = LiftState.LIFT_LOWER;
                    liftTimer.reset();
                }
                if (gamepad2.dpad_up){
                    goTo1();
                    robot.pixelHolderRotator.setPosition(robot.PIXELHOLDERROTATOR_DEPOSIT_POS);
                    liftState = LiftState.LIFT_UP;
                    liftTimer.reset();
                }
                break;

            case LIFT_HANG:
                if (gamepad2.dpad_down){
                    goTo0();
                    liftState = LiftState.LIFT_LOWER;
                    liftTimer.reset();
                }
                break;

        }

        telemetry.update();

    }

    public void goTo0() {
        double distance = 0;
        newTarget = (int) (distance *ARM_COUNTS_PER_INCH);
        robot.viperSlideLift.setTargetPosition(newTarget);
        robot.viperSlideLifttwo.setTargetPosition(newTarget);

    }
    public void goTo1(){
        double distance = 15;
        newTarget = (int) (distance *ARM_COUNTS_PER_INCH);
        robot.viperSlideLift.setTargetPosition(newTarget);
        robot.viperSlideLifttwo.setTargetPosition(newTarget);
    }

    public void hang() {
        double distance = 26;
        newTarget = (int) (distance * ARM_COUNTS_PER_INCH);
        robot.viperSlideLift.setTargetPosition(newTarget);
        robot.viperSlideLifttwo.setTargetPosition(newTarget);
    }

    @Override
    public void stop() {
    }
}