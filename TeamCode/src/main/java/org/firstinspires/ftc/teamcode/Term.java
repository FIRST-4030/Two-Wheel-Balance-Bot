package org.firstinspires.ftc.teamcode;

/**
 * Term class is used to save the Design of Experiment Term values
 */
public class Term {
    public double low; // Lowest experiment value
    public double high; // Highest experiment values
    public int n;  // Number of values from Low to High.  Minimum is 2
    private double increment; // Increment value (calculated)
    public double orig; // Place to store the original term value during the experiments
    public double current; // Place to store the current term value during the experiments
    public double min;  // Place to store a minimum response for a degree of freedom
    public double max; // Place to store a minimum response for a degree of freedom

    public Term(double lowest, double highest, int number, double original) {
        this.low = lowest;
        this.high = highest;
        this.n = number;
        if (n < 2) this.n = 2; // Minimum is 2;
        this.increment = (this.high-this.low)/(this.n-1);
        this.orig = original;
        this.current = low; // start with the low value
        resetMinMax();
    }

    //public double getIncrement() {return this.increment;}
    public int getN() {return this.n;}

    public void next() {
        double check = this.current + this.increment;
        if(check > this.high) this.current = this.low;
        else this.current = check;
    }

    //public void setOriginal() {this.current = this.orig;}
    public void resetMinMax() {
        this.min = 10000.0;
        this.max = -10000.0;
    }
    public void updateMinMax(double value){
        // build the minimum amplitude "box" on the position wave
        this.min = Math.min(value, this.min);
        this.max = Math.max(value, this.max);
    }
}
