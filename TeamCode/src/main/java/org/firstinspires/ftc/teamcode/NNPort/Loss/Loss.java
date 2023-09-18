package org.firstinspires.ftc.teamcode.NNPort.Loss;

import org.firstinspires.ftc.teamcode.NNPort.util.NDArray;

public abstract class Loss {


    public abstract double forward(NDArray<Double> expected, NDArray<Double> actual);

    public abstract NDArray<Double> back(NDArray<Double> expected, NDArray<Double> actual);


}
