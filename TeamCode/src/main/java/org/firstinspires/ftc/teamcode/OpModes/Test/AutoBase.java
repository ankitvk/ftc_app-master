package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.sun.tools.javac.code.Attribute;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Leviathan;

import ftc.library.MaelWrappers.MaelLinearOp;

public abstract class AutoBase implements Constants {
    public int programStage = 0;
    public boolean stageFinished = false;


    public void run() throws InterruptedException {
        loopStateMachine();
    }

    int nextStage = 0;
    public void nextStage(int ordinal){
        nextStage = ordinal;
        incrementStage();
    }

    public void nextStage(){
        nextStage(programStage + 1);
        System.out.println("Stage " + programStage + " Complete");
        System.out.println();
    }

    private void incrementStage(){
        programStage = nextStage;
        stageFinished = true;
    }

    private void loopStateMachine(){
        while(true){
            MainStateMachine();

            if(programStage == getOrdinalLength()) break;

        }

    }

    public abstract int getOrdinalLength();


    public void initializeStateVariables(){
        if(stageFinished) stageFinished = false;
    }

    public abstract void MainStateMachine();
}
