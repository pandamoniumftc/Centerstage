package org.firstinspires.ftc.teamcode.OffSeason.Util;

public class Toggle {
    private boolean flag = false;

    public boolean state;

    public Toggle(boolean defaultState) {
        state = defaultState;
    }

    public boolean updateState(boolean update) {
        if (update && !flag) state = !state;

        flag = update;

        return state;
    }

}
