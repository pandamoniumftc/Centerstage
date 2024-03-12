package org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem;

import android.os.Build;
import android.os.Environment;

import androidx.annotation.RequiresApi;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.Po;
import org.firstinspires.ftc.teamcode.NNPort.Data.NNData;
import org.firstinspires.ftc.teamcode.NNPort.Layer.Activation.Sigmoid;
import org.firstinspires.ftc.teamcode.NNPort.Layer.Dense;
import org.firstinspires.ftc.teamcode.NNPort.Layer.Layer;
import org.firstinspires.ftc.teamcode.NNPort.Loss.MSE;
import org.firstinspires.ftc.teamcode.NNPort.Network.Network;
import org.firstinspires.ftc.teamcode.NNPort.util.NDArray;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class PixelAI extends AbstractSubsystem {

    public Po robot;
    public File directory;

    public FileWriter writer;

    public double x;
    public double turn;

    public enum State {
        CREATE_NEW_DATA_PROMPT,
        VISUAL,
        ALIGNING,
        SAVE,
        CONTINUE_PROMPT,
        FINISHED_DATA,
        FINISHED_TRAINING
    }

    public State currentState = State.CREATE_NEW_DATA_PROMPT;

    public boolean isLoadingData = true;

    private boolean flag = false;

    private String writeString = "";

    public Network aimerAi;

    public boolean inject = false;
    public String injectString =
            "-40,0.5441744\n" +
                    "-37,0.54105752\n" +
                    "-34,0.53794064\n" +
                    "-31,0.53482376\n" +
                    "-28,0.53170688\n" +
                    "-25,0.52859\n" +
                    "-22,0.5254731199999999\n" +
                    "-19,0.5223562399999999\n" +
                    "-16,0.5192393599999999\n" +
                    "-13,0.5161224799999999\n" +
                    "-10,0.5130056\n" +
                    "-7,0.50988872\n" +
                    "-4,0.50677184\n" +
                    "-1,0.50365496\n" +
                    "1,0.5015770399999999\n" +
                    "3,0.49949911999999996\n" +
                    "5,0.49742119999999995\n" +
                    "7,0.49534327999999994\n" +
                    "9,0.49326535999999993\n" +
                    "11,0.49118744\n" +
                    "13,0.48910951999999996\n" +
                    "15,0.48703159999999995\n" +
                    "17,0.48495367999999994\n" +
                    "19,0.48287575999999993\n" +
                    "21,0.48079784\n" +
                    "23,0.47871991999999997\n" +
                    "25,0.47664199999999995\n" +
                    "27,0.47456407999999994\n" +
                    "29,0.47248615999999993\n" +
                    "31,0.4704082399999999\n" +
                    "33,0.46833031999999997\n" +
                    "35,0.46625239999999996\n" +
                    "37,0.46417447999999994\n" +
                    "39,0.46209655999999993\n";


    public PixelAI(AbstractRobot robot) {
        super(robot);
        this.robot = (Po) robot;
    }


    @Override
    public void init() {
        String userDirectory = new File("").getAbsolutePath();
        RobotLog.ii("AI buddy", userDirectory);

        directory = new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/clawAiData.txt");
        directory.setWritable(true);
        directory.setReadable(true);
        try {
            if (directory.createNewFile())  {
                RobotLog.ii("AI buddy", "created " + directory.getName());
            }
            else {
                RobotLog.ii("AI buddy", "file already existed at " + directory.getPath());
            }
        }
        catch (IOException e) {
            RobotLog.ee("AI buddy", "file failed to create at " + directory.getPath(), e.getMessage());
        }
    }

    @Override
    public void start() {

    }

    @RequiresApi(api = Build.VERSION_CODES.O)
    @Override
    public void driverLoop() {
        /*telemetry.addData("AI Buddy State", currentState.name());
        if (currentState == State.CREATE_NEW_DATA_PROMPT) {
            telemetry.addData("click B to create a new dataset, click X to train on the existing set", "(currently: " + ConeAlignerAuto.error + ")");

            if (robot.gamepad1.x) {
                currentState = State.FINISHED_DATA;
                isLoadingData = false;
                flag = true;
            }
            else if (robot.gamepad1.b) {


                currentState = State.VISUAL;
                isLoadingData = true;
            }
        }
        else if (isLoadingData) {
            if (!flag && !inject) {
                if (currentState == State.VISUAL) {
                    telemetry.addData("click A to determine nearest pixel's x pos", "(currently: " + ConeAlignerAuto.error + ")");

                    if (robot.gamepad1.a) {
                        x = ConeAlignerAuto.error;
                        robot.mecanum.drive.setPoseEstimate(new Pose2d(0, 0, 0));
                        currentState = State.ALIGNING;
                    }
                }

                else if (currentState == State.ALIGNING) {
                    telemetry.addData("click B to determine final position", "(currently: " + robot.mecanum.drive.getPoseEstimate().getHeading() + ")");

                    if (robot.gamepad1.b) {
                        turn = robot.mecanum.drive.getPoseEstimate().getHeading();
                        currentState = State.SAVE;
                    }
                    else {
                        robot.mecanum.drive.setWeightedDrivePower(
                                new Pose2d(0, 0, robot.gamepad1.right_stick_x * (robot.gamepad1.right_trigger * 0.25 + (1 - robot.gamepad1.right_trigger)))
                        );
                    }
                }

                else if (currentState == State.SAVE) {

                    writeString += x + "," + turn + "\n";

                    RobotLog.ii("AI buddy\n", writeString);

                    currentState = State.CONTINUE_PROMPT;
                }

                else if (currentState == State.CONTINUE_PROMPT) {
                    telemetry.addData("click X to finish adding data, click Y to continue", "");

                    if (robot.gamepad1.x) {

                        isLoadingData = false;
                        currentState = State.FINISHED_DATA;

                        try {
                            writer = new FileWriter(directory.getPath());
                            BufferedWriter fileWriter = new BufferedWriter(writer);
                            RobotLog.ii("AI buddy", "file writer created at " + directory.getPath());

                            try {
                                fileWriter.write(writeString);
                                RobotLog.ii("AI buddy", "logged data points:\n" + writeString);
                            } catch (IOException e) {
                                RobotLog.ee("AI buddy", "failed to write to the file");
                            }

                            try {
                                RobotLog.ii("AI buddy", "closing file writer");
                                fileWriter.close();
                                writer.close();
                                RobotLog.ii("AI buddy", "closed file writer");

                            } catch (Exception e) {
                                RobotLog.ee("AI buddy", "failed to close the file reader");
                            }

                        } catch (IOException e) {
                            RobotLog.ee("AI buddy", "file writer failed to create at " + directory.getPath());
                        }


                        RobotLog.ii("AI buddy", "read/write: " + directory.canRead() + "/" + directory.canWrite());



                    }
                    else if (robot.gamepad1.y) {
                        currentState = State.VISUAL;
                        isLoadingData = true;
                    }
                }
            }
            if (inject) {
                isLoadingData = false;
                currentState = State.FINISHED_DATA;

                try {
                    writer = new FileWriter(directory.getPath());
                    BufferedWriter fileWriter = new BufferedWriter(writer);
                    RobotLog.ii("AI buddy", "file writer created at " + directory.getPath());

                    try {
                        fileWriter.write(injectString);
                        RobotLog.ii("AI buddy", "logged data points:\n" + writeString);
                    } catch (IOException e) {
                        RobotLog.ee("AI buddy", "failed to write to the file");
                    }

                    try {
                        RobotLog.ii("AI buddy", "closing file writer");
                        fileWriter.close();
                        writer.close();
                        RobotLog.ii("AI buddy", "closed file writer");

                    } catch (Exception e) {
                        RobotLog.ee("AI buddy", "failed to close the file reader");
                    }

                } catch (IOException e) {
                    RobotLog.ee("AI buddy", "file writer failed to create at " + directory.getPath());
                }


                RobotLog.ii("AI buddy", "read/write: " + directory.canRead() + "/" + directory.canWrite());
            }
        }
        else {
            if (currentState == State.FINISHED_DATA) {
                Double[][] xTrain;
                Double[][] yTrain;

                try {
                    NNData.loadTrainingData(1, directory.getPath());
                    RobotLog.ii("AI buddy", "loaded training");
                } catch (IOException e) {
                    RobotLog.ii("im sowwy", e.getMessage());
                }

                xTrain = NNData.xTrain;
                yTrain = NNData.yTrain;

                RobotLog.ii("AI buddy", "display training");
                NNData.displayTrainingData();
                RobotLog.ii("AI buddy", "xTrain");
                for (Double[] arr : xTrain) {
                    String str = "";
                    for (int i = 0; i < arr.length - 1; i++) {
                        str += arr[i] + ", ";
                    }
                    str += arr[arr.length - 1];

                    RobotLog.ii("AI buddy", str);
                }
                RobotLog.ii("AI buddy", "yTrain");
                for (Double[] arr : yTrain) {
                    String str = "";
                    for (int i = 0; i < arr.length - 1; i++) {
                        str += arr[i] + ", ";
                    }
                    str += arr[arr.length - 1];

                    RobotLog.ii("AI buddy", str);
                }



                aimerAi = new Network(
                        new Layer[]{
                                new Dense(1, 5),
                                new Sigmoid(),
                                new Dense(5, 5),
                                new Sigmoid(),
                                new Dense(5, 5),
                                new Sigmoid(),
                                new Dense(5, 1)
                        },
                        xTrain,
                        yTrain,
                        0.05,
                        new MSE()
                );

                try {
                    aimerAi.TrainAndSave(Environment.getExternalStorageDirectory().getPath() + "/FIRST/pixelAI.txt",10000,  5000, 1000);
                    RobotLog.ii("AI buddy", "trained");


                } catch (IOException e) {
                    RobotLog.ee("AI buddy", "training failed " + e.getMessage());
                }

                for (int i = 0; i < xTrain.length; i++) {
                    RobotLog.ii("AI buddy", "passing the data: (" + NNData.asString(xTrain[i]) + "), expecting (" + NNData.asString(yTrain[i]) + "). Actual output was: " + aimerAi.predict(new NDArray<Double>(xTrain[i]), 1).printData());
                }
                currentState = State.FINISHED_TRAINING;
                RobotLog.ii("AI buddy", "complete");
            }

            if (currentState == State.FINISHED_TRAINING) {
                double turretPos = aimerAi.predict(new NDArray<Double>(new Double[] {ConeAlignerAuto.error}), 1).get(0);
                robot.feeder.intakeTurretPosition = turretPos;

                telemetry.addData("Offset: " + ConeAlignerAuto.error + " ", "Correction: " + turretPos);
            }
        }
        if (flag && !robot.gamepad1.x) flag = false;*/
    }

    @Override
    public void stop() {
        try {
            writer.close();
        }
        catch (IOException e) {
            RobotLog.ii("im sowwy", e.getMessage());
        }

    }
}
