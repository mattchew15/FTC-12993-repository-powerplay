package org.firstinspires.ftc.teamcode.Sandstorm;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Dune.DuneDrive;
import org.firstinspires.ftc.teamcode.Dune.TurretLift;

public class OuttakeSequence {

    // creating instances of each class
    ElapsedTime GlobalTimer;
    double OuttakeTimer;
    double ConeDepositTimer;
    double FlipConeTimer;
    double OuttakePickupTimer;

    DriveBase drivebase = new DriveBase();
    Inputs inputs = new Inputs();
    Outtake outtake = new Outtake();

    enum OuttakeState { // main outtake state - other fsm's will do smaller actions
        READY,
        PICKUP_EXTENDED,
        INTAKE,
        LIFT_CONE,
        CLAW_GRIP_TRANSFER,
        OUTTAKE_DEPOSIT,
        RETURN
    }
    enum ConeDepositState {
        CONE_DROP,
        BRACE_RETRACT,
        ARM_RESET,
        IDLE
    }
    enum FlipConesState {
        OUTTAKE_FLIP_READY, // should be able to transition between up and down on cone
        DROP_DOWN_ON_CONE,
        PICKUP_CONE_READY,
        IDLE
    }
    enum OuttakePickupState {
        OUTTAKE_PICKUP_READY,
        GRAB_OUTTAKE,
        HEIGHT_OUTTAKE,
        RETURN,
        IDLE
    }

    // instance of enum
    OuttakeState outtakeState = OuttakeState.READY;
    ConeDepositState coneDepositState = ConeDepositState.IDLE;
    FlipConesState flipConesState = FlipConesState.IDLE;
    OuttakePickupState outtakePickupState = OuttakePickupState.IDLE;

    // runs on init (not setup function)
    public void OuttakeHardware(){
        drivebase.Drivebase_init(hardwareMap);
        outtake.Outtake_init(hardwareMap);
    }

    // this functions runs in the setup function of the opmode (runs once instantly)
    public void OuttakeSetup(){
        // seperate timers for each state machine so no interfering (asynchronous)
        GlobalTimer = new ElapsedTime(System.nanoTime());
        GlobalTimer.reset();

        OuttakeTimer = 0; // this variable is just a normal double however it is stored as the global timer

        // hardware setup
        drivebase.motorsSetup();
        outtake.hardwareSetup();
        inputs.inputsSetup(); // this needs to be chnaged - changes toggle variables and stuff to false

        // sets the first case for the fsm to be in
        outtakeState = OuttakeState.READY;
        coneDepositState = ConeDepositState.IDLE;
        flipConesState = FlipConesState.IDLE;
        outtakePickupState = OuttakePickupState.IDLE;
    }

    public void ConeDepositSequence(){
        switch (coneDepositState) {
            case CONE_DROP:

                break;
            case CONE_DROP:

                break;
        }
    }
    public void FlipConeSequence(){

    }
    public void OuttakePickupSequence(){

    }
}
