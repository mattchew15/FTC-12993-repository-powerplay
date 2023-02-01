package org.firstinspires.ftc.teamcode.Sandstorm;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Dune.DuneDrive;
import org.firstinspires.ftc.teamcode.Dune.TurretLift;

public class OuttakeSequence {

    // creating instances of each class
    ElapsedTime GlobalTimer;
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
        ARM_RESET
    }
    enum FlipConesState {
        OUTTAKE_FLIP_READY, // should be able to transition between up and down on cone
        DROP_DOWN_ON_CONE,
        PICKUP_CONE_READY
    }
    enum OuttakePickupState {
        OUTTAKE_PICKUP_READY,
        GRAB_OUTTAKE,
        HEIGHT_OUTTAKE,
        RETURN
    }
    // runs on init (not setup function)
    public void OuttakeHardware(){
        drivebase.Drivebase_init(hardwareMap);
        outtake.Outtake_init(hardwareMap);
    }

    // this functions runs in the setup function of the opmode (runs once instantly)
    public void OuttakeSetup(){
        GlobalTimer = new ElapsedTime(System.nanoTime());
        GlobalTimer.reset();

        // hardware setup
        drivebase.motorsSetup();
        outtake.hardwareSetup();
        inputs.inputsSetup(); // this needs to be chnaged - changes toggle variables and stuff to false

        // sets the first case for the fsm to be in
        OuttakeState outtakeState = OuttakeState.READY;
        ConeDepositState coneDepositState = ConeDepositState.CONE_DROP;
        FlipConesState flipConesState = FlipConesState.OUTTAKE_FLIP_READY;
        OuttakePickupState outtakePickupState = OuttakePickupState.OUTTAKE_PICKUP_READY;
    }
}
