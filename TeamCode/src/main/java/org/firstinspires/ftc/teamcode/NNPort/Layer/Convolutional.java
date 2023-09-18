package org.firstinspires.ftc.teamcode.NNPort.Layer;


import org.firstinspires.ftc.teamcode.NNPort.util.NDArray;
import org.firstinspires.ftc.teamcode.NNPort.util.Size;

public class Convolutional extends Layer {
    public int inputDepth;
    public int inputWidth;
    public int inputHeight;
    public Size inputShape;

    public int kernelWidth;

    public Size kernelLayerShape;
    public int kernelLayerDepth;

    public Size outputShape;

    public NDArray<Double> kernels;
    public NDArray<Double> biases;

    public NDArray<Double> inputs;
    public NDArray<Double> outputs;

    public Convolutional(Size inputShape, int kernelWidth, int kernelLayerDepth) {
        this.inputDepth = inputShape.dimensions[0];
        this.inputHeight = inputShape.dimensions[1];
        this.inputWidth = inputShape.dimensions[2];
        this.inputShape = inputShape;

        this.kernelWidth= kernelWidth;
        this.kernelLayerDepth = kernelLayerDepth;

        this.kernelLayerShape = new Size(kernelLayerDepth, inputDepth, kernelWidth, kernelWidth);
        this.outputShape = new Size(kernelLayerDepth, inputHeight - kernelWidth + 1, inputWidth - kernelWidth + 1);

        this.kernels = NDArray.rand(-1, 1, kernelLayerShape);
        this.biases = NDArray.rand(-1, 1, outputShape);
        this.outputs = NDArray.rand(-1, 1, outputShape);
    }

    @Override
    public NDArray<Double> feedForward(NDArray<Double> fInput) {
        this.inputs = fInput;
        this.outputs = new NDArray(biases);

        //System.out.println("fInput: " + fInput + "\n");
        for (int d = 0; d < kernelLayerDepth; d++) {
            for (int j = 0; j < inputDepth; j++) {
                NDArray<Double> currentKernel = kernels.getSubArray(d, j);

                NDArray<Double> add = NDArray.correlate2DValid(inputs.getSubArray(j), currentKernel);

                /*for (int x = 0; x < add.length(0); x++) {
                    for (int y = 0; y < add.length(1); y++) {
                        outputs.set( (outputs.get(d, x, y) + add.get(x, y)), d, x, y);
                    }
                }*/

                //System.out.println(outputs);
                //System.out.println(add);
                //System.out.println(outputs.getSubArray(d));
                outputs.setSubArray(NDArray.add(outputs.getSubArray(d), add), d);
            }
        }

        return this.outputs;
    }

    @Override
    public NDArray<Double> backPropagate(NDArray<Double> incomingGradient, double l_rate) {
        NDArray<Double> kernelGradient = NDArray.zerosd(kernelLayerShape);
        NDArray<Double> inputGradient = NDArray.zerosd(inputShape);

        for (int i = 0; i < kernelLayerDepth; i++) {
            for (int j = 0; j < inputDepth; j++) {
                NDArray<Double> currentInputLayer = inputs.getSubArray(j);
                //System.out.println("convolution incoming gradient: " + incomingGradient);
                NDArray<Double> currentGradientLayer = incomingGradient.getSubArray(i);
                kernelGradient.setSubArray(NDArray.correlate2DValid(currentInputLayer, currentGradientLayer), i, j);

                NDArray<Double> add = NDArray.convolve2DFull(incomingGradient.getSubArray(i), kernels.getSubArray(i, j));

                for (int x = 0; x < add.length(1); x++) {
                    for (int y = 0; y < add.length(0); y++) {
                        inputGradient.set(inputGradient.get(j, y, x) + add.get(y, x), j, y, x);
                    }
                }
            }
        }

        kernels = NDArray.sub(kernels, NDArray.scalarMult(l_rate, kernelGradient));
        biases = NDArray.sub(biases, NDArray.scalarMult(l_rate, incomingGradient));

        return inputGradient;
    }
}
