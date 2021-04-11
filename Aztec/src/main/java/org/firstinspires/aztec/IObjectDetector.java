package org.firstinspires.aztec;

public interface IObjectDetector<T> {
    T objectDetected();
    void reset();
    void dispose();
}
