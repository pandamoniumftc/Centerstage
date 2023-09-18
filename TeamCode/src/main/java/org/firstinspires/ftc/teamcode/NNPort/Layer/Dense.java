package org.firstinspires.ftc.teamcode.NNPort.Layer;


import org.firstinspires.ftc.teamcode.NNPort.util.NDArray;

public class Dense extends Layer {
    public NDArray<Double> inputs;
    public NDArray<Double> outputs;

    public NDArray<Double> biases;

    public NDArray<Double> weights;

    public int inputLength;
    public int outputLength;

    public Dense(int inputLength, int outputLength) {
        this.inputLength = inputLength;
        this.outputLength = outputLength;


        this.inputs = new NDArray(inputLength);
        this.outputs = new NDArray(outputLength);

        this.weights = new NDArray(outputLength, inputLength);
        this.biases = new  NDArray(outputLength);

        for (int i = 0; i < inputLength; i++) inputs.set(0.5*(Math.random() -0.5), i);

        for (int i = 0; i < outputLength; i++) biases.set(0.5*(Math.random() - 0.5), i);

        for (int outIdx = 0; outIdx < outputLength; outIdx++) {
            for (int inIdx = 0; inIdx < inputs.length(); inIdx++) {
                this.weights.set(5*(Math.random() - 0.5), outIdx, inIdx);
            }
        }
    }


    @Override
    public NDArray<Double> feedForward(NDArray fInput) {
        //System.out.println("in: " + fInput);
        this.inputs = fInput;

        NDArray<Double> out = new NDArray(outputLength);

        for (int outIdx = 0; outIdx < outputLength; outIdx++) {
            double sum = 0;
            for (int inIdx = 0; inIdx < inputLength; inIdx++) {
                sum += weights.get(outIdx, inIdx) * inputs.get(inIdx);
            }
            out.set(sum + biases.get(outIdx), outIdx); //+biases[outIdx]
        }
        this.outputs = out;
        //System.out.println("out: " + this.outputs);
        return out;
    }

    @Override
    public NDArray<Double> backPropagate(NDArray incoming, double l_rate) {

        NDArray<Double> previousLayerGradient = new NDArray(inputLength);
        NDArray<Double> incomingGradient = (NDArray<Double>)incoming;

        for (int i = 0; i < inputLength; i++) {
            double sum = 0;
            for (int j = 0; j < outputLength; j++) {
                sum += incomingGradient.get(j) * weights.get(j, i);
            }
            previousLayerGradient.set(sum, i);
        }

        for (int i = 0; i < inputLength; i++) {
            for (int j = 0; j < outputLength; j++) {
                weights.set(weights.get(j, i) - ((incomingGradient.get(j) * inputs.get(i) * l_rate)), j, i);
            }
        }

        for (int j = 0; j < outputLength; j++) {
            biases.set(biases.get(j) - (incomingGradient.get(j) * l_rate), j);
        }

        return previousLayerGradient;

    }
}
