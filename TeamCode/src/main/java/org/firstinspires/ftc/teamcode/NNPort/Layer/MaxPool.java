package org.firstinspires.ftc.teamcode.NNPort.Layer;


import org.firstinspires.ftc.teamcode.NNPort.util.NDArray;
import org.firstinspires.ftc.teamcode.NNPort.util.Size;

public class MaxPool extends Layer {

    NDArray<Double> input;

    public Size outputSize;

    public int poolLength = 2;

    public MaxPool(int poolLength) {
        if (poolLength <= 0) throw new ArrayIndexOutOfBoundsException("Pooling dimensions must be greater than or equal than 1");

        this.poolLength = poolLength;
    }


    @Override
    public NDArray<Double> feedForward(NDArray<Double> fInput) {
        input = new NDArray<>(fInput);

        //System.out.println(fInput);

        if (fInput.dimensions.length != 3) throw new ArrayIndexOutOfBoundsException("pooling layer was given an input with invalid dimensions");
        double integerDivisionIsGross = poolLength;

        Size outShape = new Size(fInput.dimensions[0], (int)Math.ceil(fInput.dimensions[1]/integerDivisionIsGross), (int)Math.ceil(fInput.dimensions[2]/integerDivisionIsGross));
        NDArray<Double> out = NDArray.zerosd(outShape);

        for (int d = 0; d < fInput.dimensions[0]; d++) {
            for (int y = 0; y < out.dimensions[1]; y++) {
                for (int x = 0; x < out.dimensions[2]; x++) {
                    double max = fInput.get(d, y * poolLength, x * poolLength);

                    for (int dy = 0; dy < poolLength; dy++) {
                        for (int dx = 0; dx < poolLength; dx++) {
                            if ((y*poolLength + dy) < fInput.dimensions[1] && (x*poolLength + dx) < fInput.dimensions[2])
                                max = Math.max(max, fInput.get(d, y*poolLength + dy, x*poolLength + dx));

                        }
                    }
                    out.set(max, d, y, x);
                }
            }
        }

        //System.out.println(outShape);

        outputSize = outShape;
        return out;
    }

    @Override
    public NDArray<Double> backPropagate(NDArray<Double> incomingGradient, double l_rate) {
        incomingGradient.reshape(outputSize);
        NDArray<Double> inputGradient = NDArray.zerosd(new Size(input.dimensions));

        for (int d = 0; d < incomingGradient.dimensions[0]; d++) {
            for (int y = 0; y < incomingGradient.dimensions[1]; y++) {
                for (int x = 0; x < incomingGradient.dimensions[2]; x++) {
                    double max = input.get(d, y * poolLength, x * poolLength);

                    for (int dy = 0; dy < poolLength; dy++) {
                        for (int dx = 0; dx < poolLength; dx++) {
                            if ((y*poolLength + dy) < input.dimensions[1] && (x*poolLength + dx) < input.dimensions[2])
                                max = Math.max(max, input.get(d, y*poolLength + dy, x*poolLength + dx));

                        }
                    }

                    for (int dy = 0; dy < poolLength; dy++) {
                        for (int dx = 0; dx < poolLength; dx++) {
                            if ((y*poolLength + dy) < input.dimensions[1] && (x*poolLength + dx) < input.dimensions[2])
                                if (max == input.get(d, y*poolLength + dy, x * poolLength + dx))
                                    inputGradient.set(1.0 * incomingGradient.get(d, y, x), d, y * poolLength + dy, x * poolLength + dx);

                        }
                    }
                }
            }
        }

        return inputGradient;
    }


}
