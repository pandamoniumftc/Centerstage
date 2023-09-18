package org.firstinspires.ftc.teamcode.NNPort.Layer;


import org.firstinspires.ftc.teamcode.NNPort.util.NDArray;
import org.firstinspires.ftc.teamcode.NNPort.util.Size;

public class Flatten extends Layer {
    public Size inputSize;
    public Size outputSize;

    public Flatten() {

    }

    @Override
    public NDArray<Double> feedForward(NDArray<Double> fInput) {
        inputSize = new Size(fInput.dimensions);
        outputSize = new Size(fInput.volume);

        return NDArray.reshape(fInput, outputSize);
    }

    @Override
    public NDArray<Double> backPropagate(NDArray<Double> incomingGradient, double l_rate) {
        return NDArray.reshape(incomingGradient, inputSize);
    }
}
