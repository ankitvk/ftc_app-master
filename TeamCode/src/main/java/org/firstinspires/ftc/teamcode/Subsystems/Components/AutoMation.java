package org.firstinspires.ftc.teamcode.Subsystems.Components;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Leviathan;

import ftc.library.MaelControl.StateMachineRunner;
import ftc.library.MaelMotions.MaelServos.Servo.MaelServo;
import ftc.library.MaelSubsystems.MaelCollector;
import ftc.library.MaelSubsystems.MaelElevator;
import ftc.library.MaelUtils.MaelUtils;

public class AutoMation extends StateMachineRunner implements Constants {

    private Leviathan robot;
    private MaelPivot pivot;
    private MaelElevator lift;
    private MaelCollector intake;
    private MaelServo dumper;
    private double initialPivotAngle = 0;
    private double initialLiftDistance = 0;

    public AutoMation(Leviathan robot){
        this.robot = robot;
        pivot = robot.pivot;
        lift = robot.lift;
        intake = robot.intake;
        dumper = robot.index;
    }

    public enum autoStates{
        stopIntaking,
        goToDumpingPosition,
        dumpMinerals,
        goBackToInitialPosition
    }

    @Override
    public int getOrdinalLength() {
        return autoStates.values().length;
    }

    @Override
    public void MainStateMachine() {
        initializeStateVariables();

        if(programStage == autoStates.stopIntaking.ordinal()){
            initializeStateVariables();
            intake.setState(MaelCollector.State.STOP);
            dumper.setPos(.95);
            initialPivotAngle = pivot.getAngle();
            initialLiftDistance = lift.getDistance();
            nextStage();
        }

        if(programStage == autoStates.goToDumpingPosition.ordinal()){
            initializeStateVariables();
            pivot.scoringPosition();
            robot.extendDistance(-47.5);
            nextStage();
        }

        if(programStage == autoStates.dumpMinerals.ordinal()){
            initializeStateVariables();
            dumper.setPos(.15);
            MaelUtils.sleep(2);
            nextStage();
        }

        if(programStage == autoStates.dumpMinerals.ordinal()){
            initializeStateVariables();
            dumper.setPos(.15);
            MaelUtils.linearOpMode.sleep(2.5);
            nextStage();
        }

        if(programStage == autoStates.goBackToInitialPosition.ordinal()){
            initializeStateVariables();
            pivot.setAngle(initialPivotAngle);
            robot.extendDistance(initialLiftDistance);
            nextStage();
        }

        if(robot.controller.x()){
            initializeStateVariables();
            robot.stopAllSystems();
            nextStage(autoStates.goBackToInitialPosition.ordinal());
        }

    }
}
