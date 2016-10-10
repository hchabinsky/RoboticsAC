package org.firstinspires.ftc.teamcode;

/**
 * Created by scottarmstrong on 10/7/16.
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

    public int numOfColorsAboveThreshold(int redThreshold, int greenThreshold, int blueThreshold, int alphaThreshold) {

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

        return numAbove;
    }

    public int numOfColorsBelowThreshold(int redThreshold, int greenThreshold, int blueThreshold, int alphaThreshold) {

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

        return numBelow;
    }

    @Override
    public String toString() {
        return "Red: " + red + " Green: " + green + " Blue: " + blue + " Alpha: " + alpha;
    }
}
