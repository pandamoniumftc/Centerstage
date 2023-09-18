package org.firstinspires.ftc.teamcode.NNPort.Layer;


import org.firstinspires.ftc.teamcode.NNPort.util.NDArray;
import org.firstinspires.ftc.teamcode.NNPort.util.Size;

public class Reshape extends Layer {
    public Size inputShape;
    public Size outputShape;

    public Reshape(Size inputShape, Size outputShape) {
        this.inputShape = inputShape;
        this.outputShape = outputShape;
    }

    @Override
    public NDArray<Double> feedForward(NDArray<Double> fInput) {
        return NDArray.reshape(fInput, outputShape);
    }


    @Override
    public NDArray<Double> backPropagate(NDArray<Double> incomingGradient, double l_rate) {
        return NDArray.reshape(incomingGradient, inputShape);
    }
}
