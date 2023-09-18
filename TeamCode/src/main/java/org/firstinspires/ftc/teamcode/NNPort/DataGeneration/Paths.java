package org.firstinspires.ftc.teamcode.NNPort.DataGeneration;

import android.os.Environment;

public class Paths {
    public final static String RAW_TRAINING_PATH = "C:\\Users\\jackt\\IdeaProjects\\NeuralNetworksPlayground\\rawTrainingData";
    public final static String PARSED_TRAINING_PATH = Environment.getExternalStorageDirectory().getPath() + "/parsedTrainingData";

    public final static String RAW_TESTING_PATH = "C:\\Users\\jackt\\IdeaProjects\\NeuralNetworksPlayground\\rawTestingData";
    public final static String PARSED_TESTING_PATH = "C:\\Users\\jackt\\IdeaProjects\\NeuralNetworksPlayground\\parsedTestingData";

    public final static String TRAINED_NETWORKS_FILE_PATH = Environment.getExternalStorageDirectory().getPath() + "/TrainedNetworks";
}
