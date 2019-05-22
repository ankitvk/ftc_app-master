package org.firstinspires.ftc.teamcode.Control

class Toggle {
    private var currState = false
    private var prevState = false
    private var taskState = true

    fun toggle(boolState: Boolean): Boolean {
        if (boolState) {
            currState = true
        } else {
            currState = false
            if (prevState) {
                taskState = !taskState
            }
        }

        prevState = currState

        return taskState
    }
}
