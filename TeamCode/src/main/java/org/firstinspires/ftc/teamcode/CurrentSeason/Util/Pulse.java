package org.firstinspires.ftc.teamcode.CurrentSeason.Util;

public class Pulse {
    public boolean prevState;

    public Pulse() {}

    public boolean update(boolean current) {
        if (current && !prevState) {
            prevState = true;
            return true;
        }
        prevState = current;

        return false;
    }

    public boolean getState() {
        return prevState;
    }
}
