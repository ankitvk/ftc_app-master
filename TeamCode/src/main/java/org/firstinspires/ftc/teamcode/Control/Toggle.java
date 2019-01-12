package org.firstinspires.ftc.teamcode.Control;

public class Toggle {
    boolean currState = false;
    boolean prevState = false;
    boolean taskState = true;

    public boolean toggle(boolean boolState){

        if(boolState){
            currState = true;
        }

        else{
            currState = false;
            if(prevState){
                taskState = !taskState;
            }
        }

        prevState = currState;

        return taskState;
    }
}
