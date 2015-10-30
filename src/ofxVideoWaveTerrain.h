//TODO:

//consider different boundary conditions

//polyphony: deal with awkward treatment of per-agent parameters, mutex. nest agent class?

//supply function to override to change agent behavior

//it seems like reading every frame from video memory to RAM may not scale
//alternative architecture:
//manage the video volume on the GPU as a set of textures or a 3D texture
//sample by drawing small sections to a 1x1 ofPixels object

#pragma once

#include "ofMain.h"

class ofxVideoWaveTerrainAgent{
    typedef vector<ofPoint> curve;
    //using curve = vector<ofPoint>;
public:
    ofxVideoWaveTerrainAgent(double rate, double jitter);
    void draw(ofMutex &mutex, int x, int y, int w, int h);
    void update(ofMutex &mutex);
    ofPoint p,v;
    double rate, jitter;
private:
    vector<curve> history[2];
    int cur_hist;
};

class ofxVideoWaveTerrain{
    //add frames with any timestamp, query pixels at any time
public:
	ofxVideoWaveTerrain(int ftk, double ttk, int sr, double del);
    void insert_frame(ofFloatPixels &frame, double t);
    void insert_frame(ofFloatPixels &frame);
	void audioOut(float *output, int bufferSize, int nChannels);
	void draw(int x, int y, int w, int h);
	double getElapsedTime();
	void setMomentumTime(double);
	void setAudioDelay(double);
	void setAgentRate(double);
	void setPathJitter(double);
	void setFramesToKeep(int);
	void setTimeToKeep(double);
	void setAspectRatio(double);
private:
    double time_to_keep, audio_delay, momentum_time, elapsed_time; //in seconds
    double agent_rate; //in Hz
    double path_jitter; //in image widths
    double aspect_ratio;
    int sample_rate; //in Hz
    int frames_to_keep;
    map<double,ofFloatPixels> frames;
    ofFloatColor getColor(double x, double y, double t);
    ofMutex mutex;

    vector<ofxVideoWaveTerrainAgent> agents;
};
