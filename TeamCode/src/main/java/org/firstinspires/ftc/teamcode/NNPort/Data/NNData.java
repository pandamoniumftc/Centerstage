package org.firstinspires.ftc.teamcode.NNPort.Data;



import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.NNPort.DataGeneration.Paths;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Files;
import java.util.Arrays;

public class NNData {
    public static Double[][] trainingData;
    public static Double[][] xTrain;
    public static Double[][] yTrain;


    @RequiresApi(api = Build.VERSION_CODES.O)
    public static void loadTrainingData(int categories) throws IOException {
        BufferedReader reader = new BufferedReader(new FileReader(Paths.PARSED_TRAINING_PATH + "\\parsedTrainingData.txt"));
        trainingData = new Double[(int)Files.lines(java.nio.file.Paths.get(Paths.PARSED_TRAINING_PATH + "\\parsedTrainingData.txt")).count()][reader.readLine().split(",").length];
        reader.close();
        reader = new BufferedReader(new FileReader(Paths.PARSED_TRAINING_PATH + "\\parsedTrainingData.txt"));

        for (int i = 0; i < trainingData.length; i++) {
            double[] base = Arrays.stream(reader.readLine().split(",")).mapToDouble(Double::parseDouble).toArray();

            for (int j = 0; j < base.length; j++) {
                trainingData[i][j] = base[j];
            }
        }

        xTrain = new Double[trainingData.length][trainingData[0].length-1];
        yTrain = new Double[trainingData.length][categories];

        for (int i = 0; i < trainingData.length; i++) {
            for (int j = 0; j < trainingData[0].length - 1; j++) {
                xTrain[i][j] = trainingData[i][j];
            }

            for (int j = 0; j < categories; j++) {
                yTrain[i][j] = 0.0;
            }

            yTrain[i][(int)(double)trainingData[i][trainingData[0].length-1]] = 1.0;

        }

    }

    @RequiresApi(api = Build.VERSION_CODES.O)
    public static void loadTrainingDataBinaryCrossEntropy(int categories, String path) throws IOException {
        BufferedReader countingReader = new BufferedReader(new FileReader(path));
        int lines = 0;
        while (countingReader.readLine() != null) lines++;
        countingReader.close();

        BufferedReader reader = new BufferedReader(new FileReader(path));
        trainingData = new Double[lines][reader.readLine().split(",").length];
        reader.close();
        reader = new BufferedReader(new FileReader(path));

        for (int i = 0; i < trainingData.length; i++) {
            double[] base = Arrays.stream(reader.readLine().split(",")).mapToDouble(Double::parseDouble).toArray();

            for (int j = 0; j < base.length; j++) {
                trainingData[i][j] = base[j];
            }
        }

        xTrain = new Double[trainingData.length][trainingData[0].length-1];
        yTrain = new Double[trainingData.length][categories];

        for (int i = 0; i < trainingData.length; i++) {
            for (int j = 0; j < trainingData[0].length - 1; j++) {
                xTrain[i][j] = trainingData[i][j];
            }

            for (int j = 0; j < categories; j++) {
                yTrain[i][j] = 0.0;
            }

            yTrain[i][(int)(double)trainingData[i][trainingData[0].length-1]] = 1.0;

        }

    }

    @RequiresApi(api = Build.VERSION_CODES.O)
    public static void loadTrainingData(int categories, String path) throws IOException {
        BufferedReader countingReader = new BufferedReader(new FileReader(path));
        int lines = 0;
        while (countingReader.readLine() != null) lines++;
        countingReader.close();

        BufferedReader reader = new BufferedReader(new FileReader(path));
        trainingData = new Double[lines][reader.readLine().split(",").length];
        reader.close();
        reader = new BufferedReader(new FileReader(path));

        for (int i = 0; i < trainingData.length; i++) {
            double[] base = Arrays.stream(reader.readLine().split(",")).mapToDouble(Double::parseDouble).toArray();

            for (int j = 0; j < base.length; j++) {
                trainingData[i][j] = base[j];
            }
        }

        xTrain = new Double[trainingData.length][trainingData[0].length-1];
        yTrain = new Double[trainingData.length][categories];

        for (int i = 0; i < trainingData.length; i++) {
            for (int j = 0; j < trainingData[0].length - categories; j++) {
                xTrain[i][j] = trainingData[i][j];
            }

            for (int j = 0; j < categories; j++) {
                yTrain[i][j] = trainingData[i][trainingData[i].length-categories+j];
            }


        }

    }
    
    public static void displayTrainingData() {
        RobotLog.ii("AI buddy Display", "trainingData");
        for (Double[] arr : trainingData) {
            String line = "";
            for (int i = 0; i < Math.min(arr.length, 20); i++) {
                line += arr[i] + " ";
            }
            if (arr.length > 20) line += "... " + arr[arr.length-1];

            RobotLog.ii("AI buddy", line);

        }


    }

    public static String asString(Double[] arr) {
        String out = "";
        for (int i = 0; i < arr.length - 1; i++) {
            out += arr[i] + ", ";
        }
        out += arr[arr.length-1];
        return out;
    }

    public static double[][][] oneToThreeD(double[] input) {
        double[][][] out = new double[1][1][input.length];
        out[1][1] = input;

        return out;

    }

    @RequiresApi(api = Build.VERSION_CODES.O)
    public static void main(String[] args) throws IOException {
        loadTrainingData(2);
        //displayTrainingData();
    }
}
