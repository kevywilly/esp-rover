#pragma once

#include "stdio.h"
#include <math.h>

/*

 Slices
                1
            0       2
        7               3
            6       4
                5
 */

#ifndef radians
#define radians(deg) (0.01745329*deg)
#endif

#ifndef degrees
#define degrees(rad) (57.29577951*rad)
#endif

#ifndef maxof
#define maxof(a, b) (a>=b?a:b)
#endif

#ifndef signof
#define signof(x) (x<0?1:-1)
#endif

#ifndef a360
#define a360(angle) (angle>359?360-angle:(angle<0?360+angle:angle))
#endif


#ifndef map
#define map(x, in_min, in_max, out_min, out_max) ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
#endif

#define aBetween(a, b, c) (((b<c) && (a>=b && a <=c)) || ((b>c) && (a>=b || a <=c)) || (a==b && b==c))

#define angleIsZero(a) (((int)a)==0)
#define angleIs359(a) (((int)a)==359)

#define sliceOf(a1, a2) ((slice_t){a360(a1), a360(a2)})

#define SLICE_A_CLEAR 0b100
#define SLICE_B_CLEAR 0b010
#define SLICE_C_CLEAR 0b001
#define QUADRANT_CLEAR 0b111

#define PROXIMITY_CLEAR 0b111111111111



enum ReadState{WAITING_FOR_0, WAITING_FOR_359, WAITING_TO_APPLY_READINGS};
// each bit in 12 bit struct proximity_state_t represents 1 of 12 slices

#define P_HIGHEST_BIT 11
typedef struct _proximity_status_t {
    uint8_t q0: 3; // slices 2,1,0
    uint8_t q1: 3; // slices 5,4,3
    uint8_t q2: 3; // slices 8,7,6
    uint8_t q3: 3; // slices 11,10,9
} __attribute__((packed)) proximity_status_t;

typedef struct _proximity_reading_t {
    float angle;
    uint32_t distance;
    uint8_t quality;
} proximity_reading_t;

typedef struct _proximity_threshold_t {
    float angle;
    float distance;
    int slice_id;
} proximity_threshold_t;

typedef struct {
    float a1;
    float a2;
} slice_t;

static slice_t _slice_of(float v1, float v2) {
    return (slice_t) {a360(v1), a360(v2)};
}

class ProximityFrame {
public:

    static const int num_slices = 12;

    proximity_threshold_t thresholds[360];

    ProximityFrame(float xLength, float yLength, float xOffset, float yOffset, float xBuffer, float yBuffer)
            :
            _sizeX(xLength),
            _sizeY(yLength),
            _sensorXOffset(xOffset),
            _sensorYOffset(yOffset),
            _sizeXBorder(xBuffer),
            _sizeYBorder(yBuffer
            ) {
        _reset_status(_status);
        _reset_status(_status_in_progress);
        _read_state = ReadState::WAITING_FOR_0;

        _init();
    }

    ProximityFrame(float xLen, float yLen, float xOffset, float yOffset) :
            ProximityFrame(xLen, yLen, xOffset, yOffset, xLen, yLen / 2) {}

    proximity_status_t status_t() {
        proximity_status_t *t = (proximity_status_t*) &_status;

        return *t;
    }
    uint16_t status()  {
        return _status;
    }
    uint16_t status_in_progress()  {
        return _status;
    }


    slice_t quad(int index)  {
        return _quads[index];
    }

    slice_t slice(int index)  {
        return _slices[index];
    }

    int sliceForAngle(float angle) {
        for (int i = 0; i < num_slices; i++) {
            if (aBetween(a360(angle), _slices[i].a1, _slices[i].a2))
                return i;
        }
        return -1;
    }

    proximity_threshold_t thresholdForAngle(float angle)  {
        int a1 = roundf(angle);
        return thresholds[a360(a1)];
    }

    void applyReadings(proximity_reading_t * readings, size_t len) {
        _status_in_progress = PROXIMITY_CLEAR;
        for(int i=0; i < len; i++) {
            float angle = readings[i].angle;
            uint32_t distance = readings[i].distance;
            proximity_threshold_t threshold = thresholdForAngle(angle);
            if (distance < threshold.distance) {
                _status_in_progress &= ~(1 << threshold.slice_id);
            }
        }
        _status = _status_in_progress;
    }

    void applyReading(proximity_reading_t reading) {
        if(reading.distance == 0) return;

        float angle = reading.angle;
        uint32_t distance = reading.distance;

        if(angleIsZero(angle)) _status_in_progress = PROXIMITY_CLEAR;

        proximity_threshold_t threshold = thresholdForAngle(angle);

        if (distance < threshold.distance) {
            _status_in_progress &= ~(1 << threshold.slice_id);
        }

        if(angleIs359(angle)) _status = _status_in_progress;
    }

private:
    uint16_t _status;
    uint16_t _status_in_progress;

    ReadState _read_state;

    float _sizeX;
    float _sizeY;
    float _sensorXOffset;
    float _sensorYOffset;
    float _sizeXBorder;
    float _sizeYBorder;
    float _xyRatio;

    slice_t _quads[4];
    slice_t _slices[12]; // slices around the 360 circumference
    float _thetaX;  // angle width of the front and rear edges of the frame
    float _thetaY;  // angle width of the left and right edges of the frame

    void _reset_status(uint16_t &status) {
        status = PROXIMITY_CLEAR;
    }

    void _clear_status_bit(uint16_t &status, int slice_id) {
        status &= ~(1 << slice_id);
    }

    void _set_status_bit(uint16_t &status, int slice_id) {
        status |= (1 << slice_id);
    }

    void _build_slices() {
        _thetaX = 2 * fabs(degrees(atan(_xyRatio)));
        _thetaY = 180 - _thetaX;
        float delta_xx = _thetaX / 3.0;
        float delta_yy = _thetaY / 3.0;

        printf("xy-ratio %f \n", _xyRatio);
        printf("theta-xx %f -> %f\n", _thetaX, _thetaX / 6);
        printf("theta-yy %f -> %f\n", _thetaY, _thetaY / 6);

        float theta = -_thetaX / 2;
        int index = 0;
        for (int i = 0; i < 4; i++) {
            float theta0 = theta;
            float delta = ((i % 2) == 0) ? delta_xx : delta_yy;
            for (int y = 0; y < 3; y++) {
                _slices[index++] = _slice_of(theta, theta + delta);
                theta = theta + delta;
            }
            _quads[i] = _slice_of(theta0, theta);
        }
    }

    void _build_thresholds() {
        for (int i = 0; i < 360; i++) {
            float x = cos(radians((float) i));
            float y = sin(radians((float) i));
            float threshold = abs(abs(x) > abs(y)
                                  ? (signof(x) * (_sizeXBorder + _sizeX / 2) - _sensorXOffset) / x
                                  :
                                  (signof(y) * _xyRatio * (_sizeYBorder + _sizeY / 2) - _sensorYOffset + _sizeYBorder) /
                                  y);
            int slice_id = sliceForAngle(i);
            thresholds[i] = {.angle = (float) i, .distance = threshold, .slice_id = slice_id};
        }
    }

    void _init() {
        // calculate ratio of x to y so we can determine the size of each angular slice covering
        // the front and sides of the object.
        _xyRatio = (_sizeX + 2 * _sizeXBorder) / (_sizeY + 2 * _sizeYBorder);
        _build_slices();
        _build_thresholds();

    }


};

typedef ProximityFrame *ProximityFrameHandle_t;