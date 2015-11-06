//TODO:

//need to test interpolation

//currently jitter introduces noise to audio-- maybe best to eliminate jitter and rely on momentum

//consider different boundary conditions

//polyphony: deal with awkward treatment of per-agent parameters, mutex. nest agent class?

//supply function to override to change agent behavior

//it seems like reading every frame from video memory to RAM may not scale
//alternatives:
// - do everything on the GPU
// - downsample in space before reading to pixels
// - downsample in time before reading to pixels

// should each agent run its own thread, writing to a circular buffer which the audio thread reads from?
// advantages: delicious parallelism
// disadvantages: interaction between agents becomes difficult and probably inefficient
// how would that look?
// - a central data structure keeps track of how many samples behind each agent is
// - agents catch themselves up, then sleep for a few ms
// - central data structure can keep track of latencies so a good delay can be chosen

#pragma once

#include "ofMain.h"

class ofxVideoWaveTerrainAgent{
    typedef vector<ofPoint> curve;
    //using curve = vector<ofPoint>;
public:
    ofxVideoWaveTerrainAgent(double rate, double jitter, double momentum_time);
    void draw(ofMutex &mutex, int x, int y, int w, int h);
    void update(ofMutex &mutex, ofFloatColor color, double sample_rate, double aspect_ratio);
    ofPoint p,v;
    double rate, jitter, momentum_time;
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
    double time_to_keep, audio_delay, elapsed_time; //in seconds
    double base_agent_rate; //in Hz
    double aspect_ratio;
    int sample_rate; //in Hz
    int frames_to_keep;
    map<double,ofFloatPixels> frames;
    ofFloatColor getColor(double x, double y, double t);
    ofMutex mutex;

    vector<ofxVideoWaveTerrainAgent> agents;
};
