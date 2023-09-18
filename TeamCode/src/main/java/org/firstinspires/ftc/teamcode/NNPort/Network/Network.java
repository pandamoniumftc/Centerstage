package org.firstinspires.ftc.teamcode.NNPort.Network;


import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.NNPort.DataGeneration.NetworkFileManagement;
import org.firstinspires.ftc.teamcode.NNPort.Layer.Layer;
import org.firstinspires.ftc.teamcode.NNPort.Layer.LayerInterface;
import org.firstinspires.ftc.teamcode.NNPort.Loss.Loss;
import org.firstinspires.ftc.teamcode.NNPort.util.NDArray;

import java.io.IOException;
import java.util.function.Function;

public class Network {
    public Layer[] network;

    public double l_rate;

    public Double[][] xTrain;
    public Double[][] yTrain;

    public Loss loss;

    public Network(Layer[] network, Double[][] xTrain, Double[][] yTrain, double l_rate, Loss loss) {
        this.network = network;

        this.xTrain = xTrain;
        this.yTrain = yTrain;

        this.l_rate = l_rate;

        this.loss = loss;
    }

    public void Train(int epochs, int printFrequency) {
        for (int epoch = 1; epoch <= epochs; epoch++) {
            double epochError = 0;
            for (int sample = 0; sample < xTrain.length; sample++) {
                NDArray<Double> ff = new NDArray(xTrain[sample]);

                ff = predict(ff, epoch);

                //System.out.println("yTrain: " + ConvTest.printArr(yTrain[sample]));
                epochError += loss.forward(new NDArray(yTrain[sample]), ff);

                NDArray<Double> gradient = loss.back(new NDArray(yTrain[sample]), ff);
                //System.out.println(loss.getClass().getName() + "'s gradient is: " + gradient + "\n");

                for (int layerIdx = network.length - 1; layerIdx >= 0; layerIdx--) {
                    gradient = network[layerIdx].backPropagate(gradient, l_rate);

                    if (epoch % printFrequency == 0 && sample == 0)
                        System.out.println(network[layerIdx].getClass().getName() + "'s gradient is: " + gradient + "\n");
                    //System.out.println(network[layerIdx].getClass().getName() + "'s gradient is: " + gradient + "\n");
                }

            }
            if ((epoch) % printFrequency == 0 || epoch == epochs || epoch == 1)
                System.out.println(epochError/xTrain.length + ", epoch: " + epoch);
        }
    }

    public void Train(int epochs) {
        Train(epochs, 1);
    }

    public void TrainEndless(int printFrequency) {
        int epoch = 0;
        while (true) {
            double epochError = 0;
            for (int sample = 0; sample < xTrain.length; sample++) {
                NDArray<Double> ff = new NDArray(xTrain[sample]);

                ff = predict(ff, epoch);

                //System.out.println("yTrain: " + ConvTest.printArr(yTrain[sample]));
                epochError += loss.forward(new NDArray(yTrain[sample]), ff);

                NDArray<Double> gradient = loss.back(new NDArray(yTrain[sample]), ff);
                //System.out.println(loss.getClass().getName() + "'s gradient is: " + gradient + "\n");

                for (int layerIdx = network.length - 1; layerIdx >= 0; layerIdx--) {
                    gradient = network[layerIdx].backPropagate(gradient, l_rate);

                    if (epoch % printFrequency == 0 && sample == 0)
                        System.out.println(network[layerIdx].getClass().getName() + "'s gradient is: " + gradient + "\n");
                    //System.out.println(network[layerIdx].getClass().getName() + "'s gradient is: " + gradient + "\n");
                }

            }
            if ((epoch) % printFrequency == 0  || epoch == 1)
                System.out.println(epochError/xTrain.length + ", epoch: " + epoch);
            epoch++;
        }
    }

    @RequiresApi(api = Build.VERSION_CODES.O)
    public void TrainAndSave(String fileName, int saveFrequency, int printFrequency) throws IOException {
        int epoch = 0;
        while (true) {
            double epochError = 0;
            for (int sample = 0; sample < xTrain.length; sample++) {
                NDArray<Double> ff = new NDArray(xTrain[sample]);

                ff = predict(ff, epoch);

                //System.out.println("yTrain: " + ConvTest.printArr(yTrain[sample]));
                epochError += loss.forward(new NDArray(yTrain[sample]), ff);

                NDArray<Double> gradient = loss.back(new NDArray(yTrain[sample]), ff);
                //System.out.println(loss.getClass().getName() + "'s gradient is: " + gradient + "\n");

                for (int layerIdx = network.length - 1; layerIdx >= 0; layerIdx--) {
                    gradient = network[layerIdx].backPropagate(gradient, l_rate);

                    //if (epoch % saveFrequency == 0 && sample == 1)
                    //    System.out.println(network[layerIdx].getClass().getName() + "'s gradient is: " + gradient + "\n");
                }

            }
            if ((epoch) % printFrequency == 0  || epoch == 1)
                RobotLog.ii("AI buddy", epochError/xTrain.length + ", epoch: " + epoch);

            if (epoch % saveFrequency == 0) {
                NetworkFileManagement.SaveNetwork(this, fileName);

            }

            epoch++;
        }

    }

    @RequiresApi(api = Build.VERSION_CODES.O)
    public void TrainAndSave(String fileName, int epochs, int saveFrequency, int printFrequency) throws IOException {
        int epoch = 0;
        while (epoch <= epochs) {
            double epochError = 0;
            for (int sample = 0; sample < xTrain.length; sample++) {
                NDArray<Double> ff = new NDArray(xTrain[sample]);

                ff = predict(ff, epoch);

                //System.out.println("yTrain: " + ConvTest.printArr(yTrain[sample]));
                epochError += loss.forward(new NDArray(yTrain[sample]), ff);

                NDArray<Double> gradient = loss.back(new NDArray(yTrain[sample]), ff);
                //System.out.println(loss.getClass().getName() + "'s gradient is: " + gradient + "\n");

                for (int layerIdx = network.length - 1; layerIdx >= 0; layerIdx--) {
                    gradient = network[layerIdx].backPropagate(gradient, l_rate);

                    //if (epoch % saveFrequency == 0 && sample == 1)
                    //    System.out.println(network[layerIdx].getClass().getName() + "'s gradient is: " + gradient + "\n");
                }

            }
            if ((epoch) % printFrequency == 0  || epoch == 1)
                RobotLog.ii("AI buddy", epochError/xTrain.length + ", epoch: " + epoch);

            if (epoch % saveFrequency == 0) {
                NetworkFileManagement.SaveNetwork(this, fileName);

            }
            epoch++;
        }

    }

    public NDArray<Double> predict(NDArray<Double> ff, int epoch) {

        for (LayerInterface layer : network) {
            if (epoch == 0) {
                System.out.println("Attempting to feedforward through: " + layer.getClass().getName());
            }
            ff = layer.feedForward(ff);
            if (epoch == 0) {
                System.out.println("Feed forward through " + layer.getClass().getName() + " was successful!");
            }
        }

        return ff;
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public static Double[][] generateDataSet(Double[][] xTrain, Function<Double, Double> func) {
        Double[][] out = new Double[xTrain.length][xTrain[0].length];

        for (int j = 0; j < xTrain.length; j++) {
            for (int i = 0; i < xTrain[j].length; i++) {
                out[j][i] = func.apply(xTrain[j][i]);
            }
        }

        return out;
    };
    public void print2DDataSet() {
        for (Double[] arr : xTrain) {
            System.out.println("(" + arr[0] + ", " + (predict(new NDArray<Double>(arr), 1)).printData() + ")");
        }
    }



}
