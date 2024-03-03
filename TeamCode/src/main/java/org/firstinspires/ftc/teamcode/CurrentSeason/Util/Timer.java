package org.firstinspires.ftc.teamcode.CurrentSeason.Util;

public class Timer {
    private static int startTimeMs;
    public static double x;
    private static double timeDifference;
    private static int currentTimeMs;
    private static boolean isComplete;
    private static int[] arm_task;
    private static int task_id;
    private static int previous_id;

    public static int[] newArmTask(int animTime) {
        int startTimeMs = (int) System.currentTimeMillis();
        int direction;
        task_id++;

        //0 down, 1 up
        if (task_id % 2 == 0){direction = 0;}
        else {direction = 1;}

        arm_task[0] = task_id;
        arm_task[1] = direction;
        arm_task[2] = startTimeMs;

        timer(animTime);
        return arm_task;
    }

    public static double timer(int animTime) {
        currentTimeMs = (int) System.currentTimeMillis();
        task_id = arm_task[0];

        if (task_id != previous_id && startTimeMs + animTime > currentTimeMs) { isComplete = false; }
        else {
            isComplete = true;
            previous_id = task_id;
        }

        x = (double) (currentTimeMs - startTimeMs)/1000;

        if (isComplete) { return 0; }
        else {return x;}
    }
}