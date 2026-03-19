package org.firstinspires.ftc.teamcode;

/**
 * Running Average Class using a fixed array size
 */
public class RunningAverageArray {
    private final double[] window;
    private final int period;
    private int size;
    private int index;
    private double sum = 0;

    /**
     * Initializes a RunningAverage with a fixed window size (period).
     * @param period The number of elements to average.
     * @param initialize if true then set all values to zero initially, else nothing
     */
    public RunningAverageArray(int period, boolean initialize) {
        this.period = period;
        this.window = new double[period];
        this.size = 0;
        this.index = 0;

        // initialize the array to zeros if requested
        if (initialize) {
            for(int k = 0; k  < period-1; k++) {
                window[k]=0.0;
            }
            size = period;
        }
    }

    /**
     * Adds a new number to the running average calculation.
     * Operates in O(1) time.
     * @param num The new value to add.
     */
    public void add(double num) {
        // If the window is full, subtract the oldest value from the sum
        if (size == period) {
            sum -= window[index];
        } else {
            size++;
        }

        // Add the new value to the window and sum
        window[index] = num;
        sum += num;

        // Move the index, wrapping around using the modulo operator
        index = (index + 1) % period;
    }

    /**
     * Gets the current running average.
     * Operates in O(1) time.
     * @return The current average.
     */
    public double getAverage() {
        if (size == 0) {
            return 0; // Handle the case of no elements
        }
        return sum / size;
    }
}

