package org.firstinspires.ftc.teamcode.NNPort.Layer.Activation;

import android.os.Build;

import androidx.annotation.RequiresApi;

import org.firstinspires.ftc.teamcode.NNPort.Layer.ActivationLayer;

import java.util.function.Function;

@RequiresApi(api = Build.VERSION_CODES.N)
public class Sigmoid extends ActivationLayer {
    Function<Double, Double> forward = (a) -> 1.0/(1+Math.exp(-a));
    Function<Double, Double> back = (a) -> forward.apply(a) * (1.0-forward.apply(a));

    public Sigmoid() {
        super(
                (a) -> 1.0/(1+Math.exp(-a)),
                (a) -> (1.0/(1+Math.exp(-a))) * (1.0 - (1.0/(1+Math.exp(-a))))
        );
    }


}
