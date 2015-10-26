//TODO:

//add an aspect ratio parameter to compensate for non square images

//properly set sample_rate, time_to_keep, audio_delay, frames_to_keep

//consider different boundary conditions

//how best to manage timing?

//add polyphony

//supply function to override to change agent behavior

//it seems like reading every frame from video memory to RAM is not going to scale
//alternative architecture:
//manage the video volume on the GPU as a set of textures or a 3D texture
//sample by drawing small sections to a 1x1 ofPixels object

#pragma once

#include "ofMain.h"

class ofxVideoWaveTerrain{
    //add frames with any timestamp, query pixels at any time
public:
    int frames_to_keep, vertex_skip;
    double time_to_keep, audio_delay; //in seconds
    double agent_rate, agent_acc; //in Hz, Hz^2

	ofxVideoWaveTerrain(int ftk, double ttk, int sr, double del);
    void insert_frame(ofFloatPixels &frame, double t);
    void insert_frame(ofFloatPixels &frame);
	void audioOut(float *output, int bufferSize, int nChannels);
	void draw(int x, int y, int w, int h);
	double getElapsedTime();
private:
	int elapsed_samples;
	double elapsed_time;
    int sample_rate; //in Hz
    map<double,ofFloatPixels> frames;
    ofFloatColor getColor(double x, double y, double t);
    ofMutex mutex;
    ofPoint agent_coords;
    ofPoint agent_vel;

    //use lists of points to store curves
    //use lists of curves to manage wrapping around the torus
    //switch between two such lists to minimize mutex lock time
    vector<vector<ofPoint> > agent_hist[2];
    int cur_hist; 
};