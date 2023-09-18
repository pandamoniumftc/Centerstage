package org.firstinspires.ftc.teamcode.NNPort.Loss;

import org.firstinspires.ftc.teamcode.NNPort.util.NDArray;

public class BinaryCrossEntropy extends Loss {

    @Override
    public double forward(NDArray<Double> expected, NDArray<Double> actual) {


        double n = expected.length();

        double sum = 0;

        for (int i = 0; i < expected.length(); i++) {
            sum += expected.get(i)*Math.log(actual.get(i)) + (1.0-expected.get(i))*Math.log(1-actual.get(i));
        }

        sum *= -1.0/n;

        return sum;
    }

    @Override
    public NDArray<Double> back(NDArray<Double> expected, NDArray<Double> actual) {
        NDArray<Double> gradient = new NDArray<Double>(expected.length());

        for (int i = 0; i < expected.length(); i++) {
            gradient.set(((1.0/expected.length()) * (((1.0 - expected.get(i))/(1.0 - actual.get(i)) - (expected.get(i) / actual.get(i))))), i);
        }

        return gradient;
    }
}
