package org.firstinspires.ftc.teamcode.NNPort.util;

public class Size {
    public int[] dimensions;

    public Size(int... dimensions) {
        this.dimensions = dimensions;
    }

    @Override
    public String toString() {
        String out = "(";

        for (int i = 0; i < dimensions.length-1; i++) {
            out += dimensions[i] + ",";
        }
        out += dimensions[dimensions.length-1] + ")";

        return out;
    }
}
