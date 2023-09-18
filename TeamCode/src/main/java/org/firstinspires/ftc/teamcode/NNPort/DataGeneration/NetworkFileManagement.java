package org.firstinspires.ftc.teamcode.NNPort.DataGeneration;


import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.NNPort.Data.NNData;
import org.firstinspires.ftc.teamcode.NNPort.Layer.Activation.ReLU;
import org.firstinspires.ftc.teamcode.NNPort.Layer.Activation.Sigmoid;
import org.firstinspires.ftc.teamcode.NNPort.Layer.Activation.Tanh;
import org.firstinspires.ftc.teamcode.NNPort.Layer.Convolutional;
import org.firstinspires.ftc.teamcode.NNPort.Layer.Dense;
import org.firstinspires.ftc.teamcode.NNPort.Layer.Flatten;
import org.firstinspires.ftc.teamcode.NNPort.Layer.Layer;
import org.firstinspires.ftc.teamcode.NNPort.Layer.MaxPool;
import org.firstinspires.ftc.teamcode.NNPort.Layer.Reshape;
import org.firstinspires.ftc.teamcode.NNPort.Loss.BinaryCrossEntropy;
import org.firstinspires.ftc.teamcode.NNPort.Loss.Loss;
import org.firstinspires.ftc.teamcode.NNPort.Loss.MSE;
import org.firstinspires.ftc.teamcode.NNPort.Network.Network;
import org.firstinspires.ftc.teamcode.NNPort.util.NDArray;
import org.firstinspires.ftc.teamcode.NNPort.util.Size;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

public class NetworkFileManagement {
    @RequiresApi(api = Build.VERSION_CODES.O)
    public static void main(String[] args) throws IOException, ClassNotFoundException {
        sampleSave();
        sampleLoad();
    }

    @RequiresApi(api = Build.VERSION_CODES.O)
    public static void SaveNetwork(Network network, String path) throws IOException {
        RobotLog.ii("AI buddy", "creating new file at " + path);
        File file = new File(path);

        try {
            if (file.createNewFile())  {
                System.out.println("created " + file.getName());
            }
            else {
                System.out.println("file already existed at " + file.getName());
                if (file.delete()) {
                    System.out.println("file successfully deleted at " + path);
                    try {
                        if (file.createNewFile()) {
                            System.out.println("created " + file.getName());
                        }
                    }
                    catch (IOException e) {
                        System.out.println("an error occured");
                        e.printStackTrace();
                        return;
                    }
                }

                else {
                    System.out.println("file failed to delete, ending program...");
                    return;
                }
            }
        }
        catch (IOException e) {
            RobotLog.ii("AI buddy", "an error occured");
            e.printStackTrace();
        }

        RobotLog.ii("AI buddy", "created new file at " + path);
        Double[][] xTrain;
        Double[][] yTrain;

        //RobotLog.ii("AI buddy", "loading data, " + categories + " categories");
        //NNData.loadTrainingData(path, categories);
        //NNData.displayTrainingData();

        //xTrain = NNData.xTrain;
        //yTrain = NNData.yTrain;

        FileWriter writer = new FileWriter(path);

        String networkString = "";

        networkString += network.l_rate + "\n";
        networkString += network.loss.getClass().getSimpleName() + "\n\n";

        networkString += network.network.length +"\n";
        for (Layer layer : network.network) {
            networkString += layer.getClass().getSimpleName() + "/";

            if (layer.getClass().getSimpleName().equals("Reshape")) {
                Reshape reshape = (Reshape)layer;
                networkString += reshape.inputShape + ",";
                networkString += reshape.outputShape;

            }

            else if (layer.getClass().getSimpleName().equals("MaxPool")) {
                MaxPool maxPool = (MaxPool)layer;
                networkString += maxPool.poolLength;
            }

            else if (layer.getClass().getSimpleName().equals("Dense")) {
                Dense dense = (Dense)layer;
                networkString += dense.inputLength + "," + dense.outputLength + "," + dense.weights.printData() + "," + dense.biases.printData();
            }

            else if (layer.getClass().getSimpleName().equals("Convolutional")) {
                Convolutional convolutional = (Convolutional)layer;
                networkString += convolutional.inputShape + ",";
                networkString += convolutional.kernelWidth + ",";
                networkString += convolutional.kernelLayerDepth + ",";
                networkString += convolutional.kernels.printData() + ",";
                networkString += convolutional.biases.printData();
            }

            networkString += "\n";
        }

        writer.write(networkString);
        writer.close();

        return;
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public static Network loadNetwork(String fileName, Double[][] xTrain, Double[][] yTrain) throws IOException, ClassNotFoundException {
        String path = fileName;
        String currentLine;

        double learningRate;
        Loss loss;
        int layerLen;

        BufferedReader reader = new BufferedReader(new FileReader(path));

        currentLine = reader.readLine();
        learningRate = Double.parseDouble(currentLine);

        currentLine = reader.readLine();
        if (currentLine.equals("BinaryCrossEntropy")) {
            loss = new BinaryCrossEntropy();
        }
        else if (currentLine.equals("MSE")) {
            loss = new MSE();
        }
        else {
            throw new ClassNotFoundException(currentLine + "is not a valid loss function");
        }

        currentLine = reader.readLine();
        currentLine = reader.readLine();
        layerLen = Integer.parseInt(currentLine);

        System.out.println(learningRate);
        System.out.println(loss.getClass().getName());
        System.out.println(layerLen);

        Layer[] layers = new Layer[layerLen];

        for (int i = 0; i < layerLen; i++) {
            currentLine = reader.readLine();

            int splitIdx = currentLine.indexOf('/');

            String layerTypeStr = currentLine.substring(0, splitIdx);
            System.out.println(layerTypeStr);

            if (layerTypeStr.equals("ReLU")) {
                layers[i] = new ReLU();
            }
            else if (layerTypeStr.equals("Sigmoid")) {
                layers[i] = new Sigmoid();
            }
            else if (layerTypeStr.equals("Tanh")) {
                layers[i] = new Tanh();
            }
            else if (layerTypeStr.equals("Flatten")) {
                layers[i] = new Flatten();
            }

            else if (layerTypeStr.equals("Dense")) {
                String[] dataStrArr = currentLine.substring(splitIdx + 1).split(",");
                int inputLen = Integer.parseInt(dataStrArr[0]);
                int outputLen = Integer.parseInt(dataStrArr[1]);

                Double[] data = new Double[inputLen * outputLen];

                for (int j = 0; j < data.length; j++) {
                    data[j] = Double.parseDouble(dataStrArr[j+2]);
                }

                System.out.println("first: " + data[0]);
                System.out.println("last: " + data[data.length - 1]);
                System.out.println("len: " + data.length);

                Dense dense = new Dense(inputLen, outputLen);
                dense.weights = new NDArray<>(data, new Size(dense.weights.dimensions));

                Double[] data2 = new Double[outputLen];
                for (int j = 0; j < data2.length; j++) {
                    data2[j] = Double.parseDouble(dataStrArr[j+2+outputLen*inputLen]);
                }

                System.out.println("first: " + data2[0]);
                System.out.println("last: " + data2[data2.length - 1]);
                System.out.println("len: " + data2.length);

                dense.biases = new NDArray<>(data2, new Size(dense.biases.dimensions));

                layers[i] = dense;
            }
            else if (layerTypeStr.equals("Convolutional")) {
                String[] sizeStrArr = currentLine.substring(currentLine.indexOf('(')+1, currentLine.indexOf(')')).split(",");
                int[] sizeArr = new int[sizeStrArr.length];

                for (int j = 0; j < sizeArr.length; j++) {
                    sizeArr[j] = Integer.parseInt(sizeStrArr[j]);
                }

                Size size = new Size(sizeArr);

                String[] dataStrArr = currentLine.substring(currentLine.indexOf(')')+2).split(",");
                int kernelWidth = Integer.parseInt(dataStrArr[0]);
                int kernelLayerDepth = Integer.parseInt(dataStrArr[1]);

                Double[] kernelData = new Double[dataStrArr.length - 2];
                for (int j = 2; j < dataStrArr.length; j++) {
                    kernelData[j-2] = Double.parseDouble(dataStrArr[j]);
                }

                System.out.println(size);
                System.out.println(kernelWidth);
                System.out.println(kernelLayerDepth);
                System.out.println("first: " + kernelData[0]);
                System.out.println("last: " + kernelData[kernelData.length - 1]);
                System.out.println("len: " + kernelData.length);

                Convolutional conv = new Convolutional(size, kernelWidth, kernelLayerDepth);
                conv.kernels = new NDArray<>(kernelData, conv.kernelLayerShape);

                layers[i] = conv;
            }
            else if (layerTypeStr.equals("Reshape")) {
                String[] size1StrArr = currentLine.substring(currentLine.indexOf('(')+1, currentLine.indexOf(')')).split(",");
                int[] size1Arr = new int[size1StrArr.length];

                for (int j = 0; j < size1Arr.length; j++) {
                    size1Arr[j] = Integer.parseInt(size1StrArr[j]);
                }

                String[] size2StrArr = currentLine.substring(currentLine.lastIndexOf('(')+1, currentLine.lastIndexOf(')')).split(",");
                int[] size2Arr = new int[size2StrArr.length];

                for (int j = 0; j < size2Arr.length; j++) {
                    size2Arr[j] = Integer.parseInt(size2StrArr[j]);
                }

                Size size1 = new Size(size2Arr);
                Size size2 = new Size(size2Arr);

                Reshape reshape = new Reshape(size1, size2);

                layers[i] = reshape;
            }
            else if (layerTypeStr.equals("MaxPool")) {
                int poolLen = Integer.parseInt(Character.toString(currentLine.charAt(splitIdx+1)));
                System.out.println("pool Len: " + poolLen);

                MaxPool maxPool = new MaxPool(poolLen);

                layers[i] = maxPool;
            }
        }

        Network network = new Network(
                layers,
                xTrain,
                yTrain,
                //0.05,
                learningRate,
                loss
        );

        return network;
    }



    @RequiresApi(api = Build.VERSION_CODES.O)
    public static void sampleSave() throws IOException {
        Double[][] xTrain;
        Double[][] yTrain;

        NNData.loadTrainingData(4);
        //NNData.displayTrainingData();

        xTrain = NNData.xTrain;
        yTrain = NNData.yTrain;

        Layer[] layers = new Layer[] {
                new Reshape(new Size(9216), new Size(3, 48, 64)),
                new Convolutional(new Size(3, 48, 64), 7, 5),
                new ReLU(),
                new MaxPool(2),
                new Convolutional(new Size(5, 21, 29), 3, 3),
                new ReLU(),
                new MaxPool(2),
                new Flatten(),
                new Dense(3*10*14, 100),
                new Sigmoid(),
                new Dense(100, 4),
                new Sigmoid()
        };

        Network network = new Network(
                layers,
                xTrain,
                yTrain,
                //0.05,
                0.01,
                new BinaryCrossEntropy()
        );

        NetworkFileManagement.SaveNetwork(network, Paths.TRAINED_NETWORKS_FILE_PATH + "\\testing_network.txt");
    }

    @RequiresApi(api = Build.VERSION_CODES.O)
    public static Network sampleLoad() throws IOException, ClassNotFoundException {
        Double[][] xTrain;
        Double[][] yTrain;

        NNData.loadTrainingData(4);
        //NNData.displayTrainingData();

        xTrain = NNData.xTrain;
        yTrain = NNData.yTrain;

        loadNetwork(Paths.TRAINED_NETWORKS_FILE_PATH + "\\testing_network.txt", xTrain, yTrain);

        return null;
    }
}
