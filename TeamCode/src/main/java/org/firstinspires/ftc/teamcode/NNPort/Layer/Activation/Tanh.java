package org.firstinspires.ftc.teamcode.NNPort.Layer.Activation;

import org.firstinspires.ftc.teamcode.NNPort.Layer.ActivationLayer;

public class Tanh extends ActivationLayer {

    public Tanh() {
        //super( (a) -> Math.tanh(a), (a) -> 1 - (Math.pow(Math.tanh(a), 2)));
        super( (a) -> Math.tanh(a), (a) -> 1 - Math.pow(Math.tanh(a), 2));
    }



}
