package org.firstinspires.ftc.teamcode.NNPort.Layer;

import android.os.Build;

import androidx.annotation.RequiresApi;

import org.firstinspires.ftc.teamcode.NNPort.util.NDArray;

import java.util.function.Function;

public class ActivationLayer extends Layer {

    public Function<Double, Double> forward;
    public Function<Double, Double> back;

    public ActivationLayer(Function<Double, Double> forward, Function<Double, Double> back) {
        this.forward = forward;
        this.back = back;
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public NDArray<Double> feedForward(NDArray<Double> fInput) {
        NDArray<Double> out = new NDArray<>(fInput.dimensions);
        this.inputs = fInput;

        for (int i = 0; i < out.volume; i++) {
            out.set(forward.apply(fInput.get(i)), i);
        }

        this.outputs = out;

        return out;
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public NDArray<Double> backPropagate(NDArray<Double> incomingGradient, double l_rate) {
        NDArray<Double> previousLayerGradient = new NDArray<>(inputs.dimensions);

        for (int i = 0; i < previousLayerGradient.volume; i++) {

            previousLayerGradient.set(incomingGradient.get(i) * back.apply(this.inputs.get(i)), i);

        }

        return previousLayerGradient;
    };

}
