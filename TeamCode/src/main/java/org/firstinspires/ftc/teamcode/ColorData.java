package org.firstinspires.ftc.teamcode;

/**
 * @author scottarmstrong, harrisonchabinsky
 */

public class ColorData {

    private int red = 0;
    private int blue = 0;
    private int green = 0;
    private int alpha = 0;

    public ColorData(int red, int blue, int green, int alpha) {

        this.red = red;
        this.blue = blue;
        this.green = green;
        this.alpha = alpha;
    }

    public void setRed(int red) {
        this.red = red;
    }

    public void setBlue(int blue) {
        this.blue = blue;
    }

    public void setGreen(int green) {
        this.green = green;
    }

    public void setAlpha(int alpha) {
        this.alpha = alpha;
    }

    public int getRed() {
        return red;
    }

    public int getBlue() {
        return blue;
    }

    public int getGreen() {
        return green;
    }

    public int getAlpha() {
        return alpha;
    }

    /**
     * Counts how many of its fields are above inputted thresholds
     *
     * @param redThreshold
     * @param greenThreshold
     * @param blueThreshold
     * @param alphaThreshold
     * @return boolean if all the fields are above their respective thresholds
     */
    public boolean allColorsAboveThreshold(int redThreshold, int greenThreshold, int blueThreshold, int alphaThreshold) {

        int numAbove = 0;

        if (green >= greenThreshold) {
            numAbove += 1;
        }

        if (blue >= blueThreshold) {
            numAbove += 1;
        }

        if (red >= redThreshold) {
            numAbove += 1;
        }

        if (alpha >= alphaThreshold) {
            numAbove += 1;
        }

        return numAbove == 4;
    }

    /**
     * Counts how many of its fields are below inputted thresholds
     *
     * @param redThreshold
     * @param greenThreshold
     * @param blueThreshold
     * @param alphaThreshold
     * @return boolean if all the fields are below their respective thresholds
     */
    public boolean allColorsBelowThreshold(int redThreshold, int greenThreshold, int blueThreshold, int alphaThreshold) {

        int numBelow = 0;

        if (green <= greenThreshold) {
            numBelow += 1;
        }

        if (blue <= blueThreshold) {
            numBelow += 1;
        }

        if (red <= redThreshold) {
            numBelow += 1;
        }

        if (alpha <= alphaThreshold) {
            numBelow += 1;
        }

        return numBelow == 4;
    }

    @Override
    public String toString() {
        return "Red: " + red + " Green: " + green + " Blue: " + blue + " Alpha: " + alpha;
    }
}
