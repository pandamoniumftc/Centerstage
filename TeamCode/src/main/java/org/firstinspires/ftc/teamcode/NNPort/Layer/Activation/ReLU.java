package org.firstinspires.ftc.teamcode.NNPort.Layer.Activation;

import org.firstinspires.ftc.teamcode.NNPort.Layer.ActivationLayer;

public class ReLU extends ActivationLayer {
    public ReLU() {
        super(
                (a) -> Math.max(0, a),
                (a) -> ((a > 0) ? 1.0 : 0.0)
        );
    }
}
