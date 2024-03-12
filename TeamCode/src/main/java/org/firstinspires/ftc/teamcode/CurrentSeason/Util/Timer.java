package org.firstinspires.ftc.teamcode.CurrentSeason.Util;

import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.Po;

public class Timer {
    public long startTimeMs;
    public double x;
    public int[] arm_task;
    /*
    * index 0 = task_id
    * index 1 = previous_id
    * index 2 = direction
     */
    public int[] slides_task;
    /*
     * index 0 = task_id
     * index 1 = previous_id
     * index 2 = direction
     * index 3 = targetPos
     * index 4 = prevTargetPos
     */
    public boolean isComplete;

    public Timer() {
        this.arm_task = new int[3];
        this.x = 0;
        this.isComplete = false;
        this.startTimeMs = 0;
    }

    public void newArmTask(int animTime, boolean up) {
        if (this.isComplete && this.x != 0 && ((up ? 1 : 0) != this.arm_task[2])) {
            this.isComplete = false;
            this.x = 0;
        }
        if (this.arm_task[0] == this.arm_task[1] && !this.isComplete && ((up ? 1 : 0) != this.arm_task[2])) {
            this.startTimeMs = System.currentTimeMillis();
            this.arm_task[0]++;
        }

        //0 down, 1 up
        if (this.arm_task[0] % 2 == 0) {this.arm_task[2] = 0;}
        else {this.arm_task[2] = 1;}

        if (this.arm_task[0] != this.arm_task[1] && !this.isComplete) {timer(animTime);}
    }

    /*public void newSlideTask(int animTime, int targetPos) {
        if (this.isComplete && this.x != 0 && ((targetPos > this.slides_task[4] ? 1 : 0) != this.slides_task[2])) {
            this.isComplete = false;
            this.x = 0;
        }
        if (this.slides_task[0] == this.slides_task[1] && !this.isComplete && ((targetPos > this.slides_task[4] ? 1 : 0) != this.slides_task[2])) {
            this.startTimeMs = System.currentTimeMillis();
            this.slides_task[3] = targetPos;
            this.slides_task[0]++;
        }

        //0 down, 1 up
        if (this.slides_task[0] != this.slides_task[1] && targetPos < this.slides_task[4]) {this.slides_task[2] = 0;}
        else if (this.slides_task[0] != this.slides_task[1] && targetPos > this.slides_task[4]) {this.slides_task[2] = 1;}

        if (this.slides_task[0] != this.slides_task[1] && !isComplete) {timer(animTime, 1);}
    }*/

    private void timer(int animTime) {
        if (this.arm_task[0] != this.arm_task[1] && this.startTimeMs + (animTime * 1E3) > System.currentTimeMillis()) { this.isComplete = false; }
        else {
            this.isComplete = true;
            this.arm_task[1] = this.arm_task[0];
        }

        x = ((double) (System.currentTimeMillis() - this.startTimeMs) / 1000) / animTime;
                /*if (this.slides_task[0] != this.slides_task[1] && this.startTimeMs + (animTime * 1E3) > System.currentTimeMillis()) { this.isComplete = false; }
                else {
                    this.isComplete = true;
                    this.slides_task[1] = this.slides_task[0];
                }

                x = ((double) (System.currentTimeMillis() - this.startTimeMs) / 1000) / animTime;*/
    }
}