//TODO:

//currently OF_BLENDMODE_ADD doesn't work right with negative colors

//need to test interpolation

//currently jitter introduces noise to audio-- maybe best to eliminate jitter and rely on momentum

//consider different boundary conditions

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
    ofxVideoWaveTerrainAgent(double rate, double jitter, double momentum_time, ofFloatColor c);
    void draw(int x, int y, int w, int h);
    void init();
    void update(ofFloatColor color, double sample_rate, double aspect_ratio);
    void setJitter(double);
    void setRate(double);
    void setMomentumTime(double);
    ofPoint p,v;
    ofFloatColor color;
private:
    double rate, jitter, momentum_time;
    vector<curve> history[2];
    int cur_hist;
    ofMutex mutex; //agents are accessed from the video and audio threads
};

class ofxIrregularVideoVolume{
public:
    ofxIrregularVideoVolume(int ftk, double ar);
    void insert_frame(ofFloatPixels &frame, double t);
    ofFloatColor getColor(double x, double y, double t);
    void setFramesToKeep(int);
	void setAspectRatio(double);
	int getFramesToKeep();
	double getAspectRatio();
private:
    double aspect_ratio;
    int frames_to_keep;
    map<double,ofFloatPixels> frames;
    ofMutex mutex; //make the volume thread safe
};

class ofxVideoWaveTerrain{
    //add frames with any timestamp, query pixels at any time
public:
	ofxVideoWaveTerrain(int ftk, int sr, double del);
    void insert_frame(ofFloatPixels &frame);
	void audioOut(float *output, int bufferSize, int nChannels);
	void draw(int x, int y, int w, int h);
	double getElapsedTime();
	void setMomentumTime(double);
	void setAudioDelay(double);
	void setAgentRate(double);
	void setPathJitter(double);
	void scramble();
	ofxIrregularVideoVolume* getVideoVolume();
private:
    double audio_delay, elapsed_time; //in seconds
    double base_agent_rate; //in Hz
    int sample_rate; //in Hz
    vector<ofxVideoWaveTerrainAgent*> agents;
    ofxIrregularVideoVolume *ivv;
    ofMutex mutex; //needed to make elapsed_time thread safe
};
