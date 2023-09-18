package org.firstinspires.ftc.teamcode.NNPort.Layer;


import org.firstinspires.ftc.teamcode.NNPort.util.NDArray;

public class Layer implements LayerInterface {

    public NDArray<Double> inputs;
    public NDArray<Double> outputs;

    @Override
    public NDArray<Double> feedForward(NDArray<Double> fInput) {
        return new NDArray<Double>();
    }

    @Override
    public NDArray<Double> backPropagate(NDArray<Double> incomingGradient, double l_rate) { return new NDArray<Double>(); }

}
