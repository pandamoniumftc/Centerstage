package org.firstinspires.ftc.teamcode.NNPort.Layer;


import org.firstinspires.ftc.teamcode.NNPort.util.NDArray;

public interface LayerInterface {

    NDArray<Double> feedForward(NDArray<Double> fInput);

    NDArray<Double> backPropagate(NDArray<Double> incomingGradient, double l_rate);

}
