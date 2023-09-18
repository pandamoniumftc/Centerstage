package org.firstinspires.ftc.teamcode.NNPort.Loss;


import org.firstinspires.ftc.teamcode.NNPort.util.NDArray;

public class MSE extends Loss {
    public double forward(NDArray<Double> expected, NDArray<Double> actual) {

        //System.out.print("expected: " + expected + "\n");
        //System.out.print("actual: " + actual + "\n");
        double sum = 0;
        for (int i = 0; i < expected.length(); i++) {
            sum += Math.pow(expected.get(i)-actual.get(i), 2);
        }

        return sum / expected.length();
    }


    public NDArray<Double> back(NDArray<Double> expected, NDArray<Double> actual) {
        NDArray<Double> out = new NDArray(expected.length());

        //System.out.println("expected: " + expected);
        //System.out.println("actual: " + actual);

        for (int i = 0; i < expected.length(); i++) {
            out.set((2.0 / expected.length()) * (actual.get(i) - expected.get(i)), i);
        }

        //System.out.println("out:" + out);

        return out;

    }
}
