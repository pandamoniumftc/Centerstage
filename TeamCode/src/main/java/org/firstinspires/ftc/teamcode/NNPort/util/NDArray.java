package org.firstinspires.ftc.teamcode.NNPort.util;


import java.util.Arrays;
import java.util.Iterator;
import java.util.function.Function;

public class NDArray<T> implements Iterable<T> {
    public int[] dimensions;

    public T[] data;

    public final int volume;

    public NDArray(int... dimensions) {
        this.dimensions = dimensions;

        int size = 1;
        for (int d = 0; d < dimensions.length; d++) {
            size *= dimensions[d];
        }

        volume = size;

        data = (T[])(new Object[size]);
    }

    public NDArray(NDArray<T> arr) {
        dimensions = new int[arr.dimensions.length];
        for (int i = 0; i < dimensions.length; i++) {
            dimensions[i] = arr.dimensions[i];
        }

        data = (T[])(new Object[arr.data.length]);
        for (int i = 0; i < data.length; i++) {
            data[i] = arr.data[i];
        }

        this.volume = arr.volume;
    }


    public NDArray(T[] arr) {
        volume = arr.length;

        data = arr;

        dimensions = new int[] {volume};
    }

    public NDArray(T[][] arr) {
        volume = arr.length * arr[0].length;

        data = (T[])(new Object[volume]);
        

        dimensions = new int[] {arr.length, arr[0].length};
    }

    public NDArray(T[] arr, Size size) {
        volume = arr.length;

        data = arr;

        dimensions = size.dimensions;
    }

    public NDArray(NDArray<T> arr, Size size) {
        volume = arr.length();

        data = arr.data;

        dimensions = size.dimensions;
    }

    public T get(int... indices) {
        if (indices.length != dimensions.length) {
            if (indices.length == 1)
                return data[indices[0]];
            else
                throw new ArrayIndexOutOfBoundsException("amount of indices provided does not match the number of dimensions");
        }

        for (int i = 0; i < indices.length; i++) {
            if (indices[i] >= dimensions[i] || indices[i] < 0) throw new ArrayIndexOutOfBoundsException("index dimension " + i + " (value of " + indices[i] + " out of length " + dimensions[i] + ") is invalid");
        }

        int idx = 0;

        for (int d = dimensions.length-1; d >= 0; d--) {
            int coff = 1;

            for (int cd = dimensions.length-1; cd > d; cd--) {
                coff *= dimensions[cd];
            }

            idx += indices[d] * coff;
        }

        return data[idx];
    }

    public NDArray<T> getSubArray(int... indices) {
        if (indices.length >= dimensions.length || indices.length == 0) throw new ArrayIndexOutOfBoundsException("array does not have a sub array of the implicated size. The length of the indices array provided (" + indices.length + ") is greater than or equal to the length of the dimensions array (" + dimensions.length + ") or equal to zero which is " + ((indices.length == 0) ? "the " : "not the ") + "case in this instance");
        for (int i = 0; i< indices.length; i++) {
            if (indices[i] < 0 || indices[i] >= dimensions[i]) throw new ArrayIndexOutOfBoundsException("index provided does not fit in the dimensions of the arr ay");
        }

         int size = 1;
         for (int i = dimensions.length-1; i >= indices.length; i--) {
             size *= dimensions[i];
         }
         //System.out.println("sub array is going to have a volume of " + size);

         int[] beginningIndices = new int[dimensions.length];

         for (int i = 0; i < indices.length; i++) {
             beginningIndices[i] = indices[i];
         }

         int idx = 0;

        for (int d = dimensions.length-1; d >= 0; d--) {
            int coff = 1;

            for (int cd = dimensions.length-1; cd > d; cd--) {
                coff *= dimensions[cd];
            }

            idx += beginningIndices[d] * coff;

        }

        //System.out.println("sub array begins at index " + idx + " in the original array");

        T[] subData = (T[])(new Object[size]);

        //System.out.println("data len " + data.length + "\n idx: " + idx + "\nsize: " + size);
        for (int i = 0; i < size; i++) {
            subData[i] = data[idx + i];
        }

        int[] newDims = new int[dimensions.length - indices.length];


        for (int i = 0; i < newDims.length; i++) {
            newDims[i] = dimensions[i + (indices.length)];
        }

        return new NDArray<T>(subData, new Size(newDims));
    }

    public void setSubArray(NDArray<T> arr, int... indices) {
        if (indices.length >= dimensions.length || indices.length == 0) throw new ArrayIndexOutOfBoundsException("array does not have a sub array of the implicated size. The length of the indices array provided (" + indices.length + ") is greater than or equal to the length of the dimensions array (" + dimensions.length + ") or equal to zero which is " + ((indices.length == 0) ? "the " : "not the ") + "case in this instance");
        for (int i = 0; i< indices.length; i++) {
            if (indices[i] < 0 || indices[i] >= dimensions[i]) throw new ArrayIndexOutOfBoundsException("index provided does not fit in the dimensions of the arr ay");
        }

        int size = 1;
        for (int i = dimensions.length-1; i >= indices.length; i--) {
            size *= dimensions[i];
        }
        //System.out.println("sub array is going to have a volume of " + size);

        int[] beginningIndices = new int[dimensions.length];

        for (int i = 0; i < indices.length; i++) {
            beginningIndices[i] = indices[i];
        }

        int idx = 0;

        for (int d = dimensions.length-1; d >= 0; d--) {
            int coff = 1;

            for (int cd = dimensions.length-1; cd > d; cd--) {
                coff *= dimensions[cd];
            }

            idx += beginningIndices[d] * coff;
        }

        for (int i = idx; i < idx + size; i++) {
            data[i] = arr.get(i-idx);
        }
    }

    public void set(T value, int... indices) {
        if (indices.length != dimensions.length) {
            if (indices.length == 1) {
                data[indices[0]] = value;
                return;
            }
            else
                throw new ArrayIndexOutOfBoundsException("amount of indices provided does not match the number of dimensions");
        }

        for (int i = 0; i < indices.length; i++) {
            if (indices[i] >= dimensions[i] || indices[i] < 0) throw new ArrayIndexOutOfBoundsException("index dimension " + i + " (value of " + indices[i] + " out of length " + dimensions[i] + ") is invalid");
        }

        int idx = 0;

        for (int d = dimensions.length-1; d >= 0; d--) {
            int coff = 1;

            for (int cd = dimensions.length-1; cd > d; cd--) {
                coff *= dimensions[cd];
            }

            idx += indices[d] * coff;
        }

        data[idx] = value;
    }

    public void reshape(Size newDimensions) {
        int size = 1;
        for (int i = 0; i < newDimensions.dimensions.length; i++) {
            size *= newDimensions.dimensions[i];
        }
        if (size != volume) {
            throw new ArrayIndexOutOfBoundsException("dimensions provided do not have the same volume, original: " + volume + ", provided : " + size);
        }
        dimensions = newDimensions.dimensions;
    }

    public String printData() {
        String out = "";

        for (int i = 0; i < data.length-1; i++) {
            out += data[i] + ",";
        }
        out += data[data.length-1];

        return out;
    }

    public static NDArray reshape(NDArray arr, Size size) {
        return new NDArray(arr, size);
    }

    public static NDArray<Double> zerosd(Size shape) {
        int size = 1;

        for (int i = 0; i < shape.dimensions.length; i++) {
            size *= shape.dimensions[i];
        }

        Double[] data = new Double[size];

        for (int i = 0; i < size; i++) {
            data[i] = 0.0;
        }

        return new NDArray<Double>(data, shape);
    }

    public static NDArray<Double> lambda(Function<Double, Double> f, int... dimensions) {
        int size = 1;
        for (int i = 0; i < dimensions.length; i++) {
            size *= dimensions[i];
        }


        Double[] data = new Double[size];
        for (int i = 0; i < size; i++) {
            data[i] = f.apply((double)i);
        }

        return new NDArray<Double>(data, new Size(dimensions));
    }

    public static NDArray<Double> rand(double min, double max, int... dimensions) {
        NDArray arr = new NDArray<Double>(dimensions);
        double range = max-min;
        for (int i = 0; i < arr.volume; i++) {
            arr.data[i] = Math.random()*range - min;
        }

        return arr;
    }

    public static NDArray<Double> rand(double min, double max, Size shape) {
        return NDArray.rand(min, max, shape.dimensions);
    }

    public static NDArray<Double> correlate2DValid(NDArray<Double> src, NDArray<Double> kernel) {
        if (src.dimensions.length != 2) throw new ArrayIndexOutOfBoundsException("source has " + src.dimensions.length + " dimensions instead of 2");
        if (kernel.dimensions.length != 2) throw new ArrayIndexOutOfBoundsException("kernel has " + kernel.dimensions.length + " dimensions instead of 2");

        //System.out.println(src.length(0)-kernel.length(0)+1 + "    " +(src.length(1)-kernel.length(1)+1));
        //System.out.println(src.length(1) + " " + kernel.length(1));
        NDArray<Double> output = new NDArray(src.length(0)-kernel.length(0)+1, src.length(1)-kernel.length(1)+1);


        for (int y = 0; y < output.length(0); y++) {
            for (int x = 0; x < output.length(1); x++) {
                double val = 0;

                for (int j = 0; j < kernel.length(0); j++) {
                    for (int i = 0; i < kernel.length(1); i++) {
                        val += src.get(y+j,x+i) * kernel.get(j, i);
                    }
                }

                output.set(val, y, x);
            }
        }

        return output;
    }

    public static NDArray<Double> rot1802D(NDArray<Double> arr) {
        if (arr.dimensions.length != 2) throw new ArrayIndexOutOfBoundsException("array has " + arr.dimensions.length + " dimensions instead of 2");
        NDArray<Double> out = new NDArray<>(arr.length(0), arr.length(1));

        for (int i = out.length(0)-1; i >= 0; i--) {
            for (int j = out.length(1)-1; j >= 0; j--) {
                //System.out.println(i + ", " + (kernel.length - (i+1)));
                //System.out.println(j + ", " + (kernel[0].length - (j+1)));
                out.set(arr.get(arr.length(0) - (i+1), arr.length(1) - (j+1)), i, j);
            }
        }

        return out;
    }

    public static NDArray<Double> convolve2DFull(NDArray<Double> input, NDArray<Double> kernel) {
        NDArray<Double> output = new NDArray<>(input.length(0)+kernel.length(0)-1, input.length(1)+kernel.length(1)-1);

        kernel = rot1802D(kernel);

        for (int y = -kernel.length(0)+1; y < input.length(0); y++) {
            for (int x = -kernel.length(1)+1; x < input.length(1); x++) {
                double val = 0;

                for (int j = 0; j < kernel.length(0); j++) {
                    for (int i = 0; i < kernel.length(1); i++) {
                        //System.out.println("(" + y + ", " + x + ")  " + "(" + (y+j) + ", " + (x+i) + ") = " + input[y+j][x+i]);
                        if ( (y+j >= 0 && y+j < input.length(0)) && (x+i >= 0 && x+i < input.length(1))) {

                            val += input.get(y+j, x+i) * kernel.get(j, i);
                        }

                    }
                }

                //System.out.println(y+kernel.length-1);
                //System.out.println(x+kernel[0].length-1);
                output.set(val, y+kernel.length(0)-1, x+kernel.length(1)-1);
                //System.out.println();
            }
        }

        return output;
    }

    public static NDArray<Double> scalarMult(double scalar, NDArray<Double> arr) {
        NDArray<Double> out = new NDArray<>(arr, new Size(arr.dimensions));

        for (int i = 0; i < out.volume; i++) {
            out.set(arr.get(i) * scalar, i);
        }

        return out;
    }

    public static NDArray<Double> add (NDArray<Double> addend1, NDArray<Double> addend2) {

        if (!Arrays.equals(addend1.dimensions, addend2.dimensions)) throw new ArrayIndexOutOfBoundsException("arrays do not have matching dimensions");

        NDArray<Double> out = new NDArray<>(addend1.dimensions);

        for (int i = 0; i < out.volume; i++) {
            out.set((addend1.get(i) + addend2.get(i)), i);
        }

        return out;
    }

    public static NDArray<Double> sub (NDArray<Double> minuend, NDArray<Double> subtrahend) {
        if (!Arrays.equals(subtrahend.dimensions, subtrahend.dimensions)) throw new ArrayIndexOutOfBoundsException("arrays do not have matching dimensions");

        NDArray<Double> out = new NDArray<>(minuend.dimensions);

        for (int i = 0; i < out.volume; i++) {
            out.set((minuend.get(i) - subtrahend.get(i)), i);
        }

        /*NDArray<Double> out = NDArray.zerosd(new Size(minuend.dimensions));

        for (int i = 0; i < out.volume; i++) {
            out.data[i] = minuend.data[i] - subtrahend.data[i];
        }*/

        return out;
    }

    public static Double max(NDArray<Double> arr) {
        double max = arr.get(0);
        for (int i = 1; i < arr.volume; i++) {
            max = Math.max(max, arr.get(i));
        }

        return max;
    }

    public int length(int dimensionIndex) {
        if (dimensionIndex < 0 || dimensionIndex > dimensions.length) {
            throw new ArrayIndexOutOfBoundsException("dimension index " + dimensionIndex + " (value of " + dimensionIndex + " out of length " + dimensions.length + ") is invalid");
        }

        return dimensions[dimensionIndex];
    }

    public int length() {
        return volume;
    }

    private String printArr(Object[] arr) {
        String out = "";
        for (int i = 0; i < arr.length-1 ; i++) {
            out += arr[i].toString() + ", ";
        }
        out += arr[arr.length-1];
        return out;
    }

    private String printArr(int[] arr) {
        String out = "";
        for (int i = 0; i < arr.length-1 ; i++) {
            out += arr[i] + ", ";
        }
        out += arr[arr.length-1];
        return out;
    }

    @Override
    public String toString() {
        return "NDArray {" +
                "\n\tdimensions: " + printArr(dimensions) +
                "\n\tdata: " + printArr(data) +
                "\n\tvolume: " + volume +
                "\n}";
    }

    @Override
    public Iterator<T> iterator() {
        return new MyIterator();
    }

    class MyIterator implements Iterator<T> {

        private int index = 0;

        public boolean hasNext() {
            return index < volume;
        }

        public T next() {
            return data[index++];
        }

        public void remove() {
            throw new UnsupportedOperationException("not supported yet");
        }
    }
}
