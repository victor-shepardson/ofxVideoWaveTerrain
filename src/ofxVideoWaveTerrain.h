//TODO:

//currently OF_BLENDMODE_ADD doesn't work right with negative colors
// is oF clamping negative colors somewhere?

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
#include <array>
#include <atomic>

//quick and dirty multichannel ring buffer
template <unsigned int C, unsigned int L>
class ofxDelayLine{
public:
    ofxDelayLine(){
        head = 0;
    }
    void insert(const array<double, C> v){
        head = (head+1)%L;
        d[head] = v;
    }
    double get(unsigned int t, unsigned int c){
        return d[(head-t+L)%L][c%C];
    }
    array<double, C> get(unsigned int t){
        return d[(head-t+L)%L];
    }
private:
    unsigned int head;
    array< array<double, C>, L > d;
};

//video volume supporting linear interpolation
//supports irregular sampling in time and dynamic resolution
//thread safe read/write
class ofxIrregularVideoVolume{
public:
    ofxIrregularVideoVolume(int ftk, double ar);
    void insert_frame(shared_ptr<ofFloatPixels> &frame, double t);
    ofFloatColor getColor(double x, double y, double t);
    void setFramesToKeep(int);
	void setAspectRatio(double x){ aspect_ratio = x; }
	int getFramesToKeep(){ return frames_to_keep; }
	double getAspectRatio(){ return aspect_ratio; }
private:
    ofFloatColor getColorFromFrames(double, double, double, shared_ptr<ofFloatPixels>&, shared_ptr<ofFloatPixels>&);
    ofFloatColor getColorFromFrame(double, double, shared_ptr<ofFloatPixels>&);
    double t_last_warned, t_cache_before, t_cache_after;
    atomic<double> aspect_ratio;
    shared_ptr<ofFloatPixels> pix_cache_before, pix_cache_after;
    int frames_to_keep;
    map<double, shared_ptr<ofFloatPixels> > frames;
    ofMutex mutex; //make the volume thread safe
};

class ofxVideoWaveTerrainAgent{
    typedef vector<ofPoint> curve;
    //using curve = vector<ofPoint>;
public:
    ofxVideoWaveTerrainAgent();
    void draw(int x, int y, int w, int h);
    void init();
    void update(ofFloatColor color, double sample_rate, double aspect_ratio);
    void randomColorWithAlpha(double alpha);
    void randomRotation();
    void setMomentumTime(double);
    atomic<double> rate, jitter, rotation, comb_freq_v, comb_freq_p, comb_fb_v, comb_fb_p;
    int sample_rate;
    ofPoint p,v,lp,peak;
    ofFloatColor color;
    ofBlendMode blend_mode;
private:
    vector<curve> history[2]; //two buffers of curves: one to write to in audio thread, one to read from in video thread
    ofxDelayLine<2, 48000> v_history;
    ofxDelayLine<2, 48000> p_history;
    int cur_hist;
    double epsilon;
    ofMutex mutex; //agents are accessed from the video and audio threads, need to lock around history
};

class ofxVideoWaveTerrain{
    //add frames with any timestamp, query pixels at any time
public:
	ofxVideoWaveTerrain(size_t n_agents, size_t ftk, size_t sr, double del);
	~ofxVideoWaveTerrain();
    void insert_frame(shared_ptr<ofFloatPixels> &frame);
	void audioOut(float *output, int bufferSize, int nChannels);
	void draw(int x, int y, int w, int h);
	double getElapsedTime(){ return elapsed_time; }
    double getAudioDelay(){ return audio_delay; }
	void setMomentumTime(double x, double scale = 1);
	void setAudioDelay(double);
	void setAgentRate(double x, double scale = 1);
	void setAgentCombFrequencyP(double x, double scale = 1);
    void setAgentCombFrequencyV(double x, double scale = 1);
    void setAgentCombFeedbackP(double);
    void setAgentCombFeedbackV(double);
	void setPathJitter(double);
	void scramble();
	ofxIrregularVideoVolume* getVideoVolume(){ return ivv; }
private:
    atomic<double> audio_delay, elapsed_time; //in seconds
    atomic<uint32_t> sample_rate; //in Hz
    
    vector<ofxVideoWaveTerrainAgent*> agents;
    ofxIrregularVideoVolume *ivv;
    // ofMutex mutex; //thread safety is outsourced to ofxIrregularVideoVolume and ofxVideoWaveTerrainAgent,
    //  except for a few atomics
};
