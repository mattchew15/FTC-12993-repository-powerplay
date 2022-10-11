package org.firstinspires.ftc.teamcode;

//@disabled

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TinzenSlide")
public class TinzenSlide extends LinearOpMode {

    private DcMotor FR;
    private DcMotor BR;
    private DcMotor Intake;
    private Servo LeftMoveHorizontal;
    private Servo RightMoveHorizontal;
    private Servo LeftMoveBucket;
    private Servo RightMovebucket;
    private Servo MoveOutake;
    private Servo CapServo;
    private DcMotor BL;
    private DcMotor FL;
    private DcMotor Slides;
    private DcMotor Carousel;
    ColorSensor FreightDetect;

    ElapsedTime Timer;
    double PowerBase;
    double PowerBaseTurn;
    double PowerStrafe;
    double PowerSliderUp;
    double PowerSliderDown;
    double PowerCarousel;
    double DirectionBase;
    double DirectionStrafe;

    private void Setup() {
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        PowerBase = 1;
        PowerBaseTurn = 0.55;
        PowerStrafe = 1;
        PowerSliderUp = 0.7;
        PowerSliderDown = -0.5;
        PowerCarousel = 0.5;
        DirectionBase = 1;
        DirectionStrafe = 1;
        Timer = new ElapsedTime(System.nanoTime());
        Timer.reset();
        Slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public int lastError = 0;
    public int integral = 0;
    public int target = 0;
    public int slideDirection = 0;
    public int carouselDirection = 0;
    public double TimerCarousel = 0;
    public double IntakeTimer = 0;
    public double IntakeDirection = 0;
    public boolean Toggled = false;
    public boolean Toggledd = false;

    @Override
    public void runOpMode() {

        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        LeftMoveHorizontal = hardwareMap.get(Servo.class, "LeftMoveHorizontal");
        RightMoveHorizontal = hardwareMap.get(Servo.class, "RightMoveHorizontal");
        LeftMoveBucket = hardwareMap.get(Servo.class, "LeftMoveBucket");
        RightMovebucket = hardwareMap.get(Servo.class, "RightMovebucket");
        MoveOutake = hardwareMap.get(Servo.class, "MoveOutake");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FL = hardwareMap.get(DcMotor.class, "FL");
        Slides = hardwareMap.get(DcMotor.class, "Slides");
        Carousel = hardwareMap.get(DcMotor.class, "Carousel");
        CapServo = hardwareMap.get(Servo.class, "MoveSEGrabber");
        FreightDetect = hardwareMap.get(ColorSensor.class, "FreightDetect");

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        if (opModeIsActive()) {

            Setup();


            while (opModeIsActive()) {

                Carousel(gamepad1.dpad_right, gamepad1.dpad_left, gamepad1.right_trigger, gamepad1.left_trigger);
                Slides(gamepad2.left_bumper, gamepad2.right_bumper);
                SlidesMid(gamepad2.dpad_right);
                Slidescap(gamepad2.b, gamepad2.a);
                Strafe(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                dump(gamepad1.right_bumper);
                IntakeSpin(gamepad2.right_trigger - gamepad2.left_trigger, 1500, 1);
                CapElement(gamepad2.dpad_up,gamepad2.dpad_left,gamepad2.dpad_down);
                SlidesControl(gamepad2.x, gamepad2.y);
                PowerToggle(gamepad1.a);
                DirectionToggle(gamepad1.b);
                telemetry.update();
                telemetry.addData("Red", FreightDetect.red());
                telemetry.addData("Blue", FreightDetect.blue());
                telemetry.addData("Green", FreightDetect.green());
                }
            }
        }

    public double PID(int error, double kp, double ki, double kd) {
        integral = integral + error;
        return(kp * error + (lastError - error) * kd + integral * ki);
    }

    public void PowerToggle(boolean toggle) {
        if (toggle) {
            if (!Toggled) {
                if (PowerBase == 1) {
                    PowerBase = 0.25;
                    PowerBaseTurn = 0.25;
                    PowerStrafe = 0.25;
                } else {
                    PowerBase = 1;
                    PowerBaseTurn = 0.75;
                    PowerStrafe = 1;
                }
                Toggled = true;
            }
        }
        else {
            Toggled = false;
        }
    }
    public void DirectionToggle(boolean togglee) {
        if (togglee) {
            if (!Toggledd) {
                if (DirectionBase == 1) {
                    DirectionBase = -1;
                    DirectionStrafe = -1;
                } else {
                    DirectionBase = 1;
                    DirectionStrafe = 1;
                }
                Toggledd = true;
            }
        }
        else {
            Toggledd = false;
        }
    }

    public void IntakeSpin(double direction, int duration, double power) {

        if (IntakeTimer == 0) {
            if (FreightDetect.red() > 200) {
                IntakeTimer = Timer.milliseconds() + 1500;
            }
        }
        if (Timer.milliseconds() > IntakeTimer) {
            IntakeDirection = direction;
            if (IntakeDirection != 0) {
                IntakeTimer = 0;
            }
            Intake.setPower(power * IntakeDirection);
        }
        else {
            Intake.setPower(power * -1);
        }
    }

    public void SlidesControl(boolean down, boolean up) {
        if (down) {
            slideDirection = 3;
            Slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Slides.setPower(-0.1);
        }
        else if (up) {
            slideDirection = 3;
            Slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Slides.setPower(0.2);
        }
        else if (slideDirection == 3) {
            Slides.setPower(0);
            slideDirection = 0;
        }
    }

    public void OuttakeIn() {

        LeftMoveHorizontal.setPosition(0.0685);
        RightMoveHorizontal.setPosition(0.46);
        LeftMoveBucket.setPosition(0.83);
        RightMovebucket.setPosition(0.04);
        MoveOutake.setPosition(0.59);
        CapServo.setPosition(0.76);
        }

    public void OuttakeOut(){
        LeftMoveHorizontal.setPosition(0.515);
        RightMoveHorizontal.setPosition(0.02);
        RightMovebucket.setPosition(0.57);
        LeftMoveBucket.setPosition(0.3);
        MoveOutake.setPosition(0.45);
        CapServo.setPosition(0.76);
    }

    public void dump(boolean dump){

        if (dump) {
            MoveOutake.setPosition(0.74);
        }
    }

    public void CapElement(boolean up, boolean cap, boolean down){
        if(up){
            CapServo.setPosition(0.76);
        }
        if(cap){
            CapServo.setPosition(0.55);
        }
        if(down){
            CapServo.setPosition(0.25);
        }
    }

    public void Strafe(float LY, float LX, float RX) {
        BL.setPower((LY * (PowerBase - (LX*0.5)) * DirectionBase) - (RX * PowerBaseTurn - LX * PowerStrafe * DirectionStrafe));
        FL.setPower((LY * (PowerBase - (LX*0.5)) * DirectionBase) - (RX * PowerBaseTurn + LX * PowerStrafe * DirectionStrafe));
        BR.setPower((LY * (PowerBase - (LX*0.5)) * DirectionBase) + (RX * PowerBaseTurn - LX * PowerStrafe * DirectionStrafe));
        FR.setPower((LY * (PowerBase - (LX*0.5)) * DirectionBase) + RX * PowerBaseTurn + LX * PowerStrafe * DirectionStrafe);
    }

    public void Carousel(boolean right, boolean left, float manual, float manual1) {

        if (right) {
            TimerCarousel = Timer.milliseconds();
            carouselDirection = 1;
        }
        if (left) {
            TimerCarousel = Timer.milliseconds();
            carouselDirection = 2;
        }
        if (manual > 0.1) {
            Carousel.setPower(-manual+manual1);
        }
        else if (carouselDirection != 0) {
            if (Timer.milliseconds() - TimerCarousel < 1250){
                if (carouselDirection == 1) {
                    if (Timer.milliseconds() - TimerCarousel < 300) {
                        Carousel.setPower(0.4);
                    } else if (Timer.milliseconds() - TimerCarousel < 600) {
                        Carousel.setPower(0.4 + 0.6 * (((Timer.milliseconds() - TimerCarousel) - 300) / 300));
                    } else if (Timer.milliseconds() - TimerCarousel > 850) {
                        Carousel.setPower(-0.1);
                    }

                    else {
                        Carousel.setPower(1);
                    }
                } else {
                    if (Timer.milliseconds() - TimerCarousel < 300) {
                        Carousel.setPower(-0.4);
                    } else if (Timer.milliseconds() - TimerCarousel < 600) {
                        Carousel.setPower(-0.4 - 0.6 * (((Timer.milliseconds() - TimerCarousel) - 300) / 300));
                    } else if (Timer.milliseconds() - TimerCarousel > 850) {
                    Carousel.setPower(0.1);
                    }

                    else {
                        Carousel.setPower(-1);
                    }
                }
            }
            else {
                carouselDirection = 0;
            }
        }
        else {
            Carousel.setPower(0);
        }
    }

    private void Slides(boolean up, boolean down) {
        telemetry.addData("position", Slides.getCurrentPosition());
        if (up) {
            slideDirection = 1;
            OuttakeOut();
            IntakeSpin(2,1000,1);
        } else if (down) {
            slideDirection = 2;
            OuttakeIn();
        }

        if (slideDirection == 1) {
            if (Slides.getCurrentPosition() < 1700) {
                Slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                Slides.setTargetPosition(2050);
                Slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            Slides.setPower(1);
        } else if (slideDirection == 2) {
            if (Slides.getCurrentPosition() > 350) {
                Slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                Slides.setTargetPosition(0);
                Slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (Slides.getCurrentPosition() <= 0) {
                  //  IntakeSpin(1, 100, 1);
                }
            }
            Slides.setPower(-0.75);
        }
    }
    private void SlidesMid(boolean up) {
        if (up) {
            OuttakeOut();
            Slides.setTargetPosition(1000);
            Slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slides.setPower(1);
        }
    }

    private void Slidescap(boolean up, boolean down) {
        telemetry.addData("position", Slides.getCurrentPosition());
        if (up) {
            slideDirection = 1;
            CapServo.setPosition(0.55);
        } else if (down) {
            slideDirection = 2;
            CapServo.setPosition(0.76);

        }

        if (slideDirection == 1) {
            if (Slides.getCurrentPosition() < 1700) {
                Slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                Slides.setTargetPosition(2050);
                Slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            Slides.setPower(1);
        } else if (slideDirection == 2) {
            if (Slides.getCurrentPosition() > 350) {
                Slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                Slides.setTargetPosition(0);
                Slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (Slides.getCurrentPosition() <= 0) {
                    //  IntakeSpin(1, 100, 1);
                }
            }
            Slides.setPower(-0.75);
        }
    }
}