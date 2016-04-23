#include "ofxVideoWaveTerrain.h"

#include <xmmintrin.h>

template<unsigned int C, unsigned int L>
ofxDelayLine<C, L>::ofxDelayLine(){
    head = 0;
}

template<unsigned int C, unsigned int L>
void ofxDelayLine<C, L>::insert(const array<double, C> v){
    head = (head+1)%L;
    d[head] = v;
}

template<unsigned int C, unsigned int L>
double ofxDelayLine<C, L>::get(unsigned int t, unsigned int c){
    return d[(head-t+L)%L][c%C];
}

template<unsigned int C, unsigned int L>
array<double, C> ofxDelayLine<C, L>::get(unsigned int t){
    return d[(head-t+L)%L];
}

ofxVideoWaveTerrain::~ofxVideoWaveTerrain(){
    delete ivv;
    for(auto i = agents.begin(); i!=agents.end(); i++)
        delete (*i);
}

ofxVideoWaveTerrain::ofxVideoWaveTerrain(size_t n_agents=16, size_t ftk=48, size_t sr=48000, double del=.2){

    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);

    ivv = new ofxIrregularVideoVolume(ftk, 1);

	sample_rate = sr;
	elapsed_time = 0;
	audio_delay = del;

    for(size_t i=0; i<n_agents; i++){
        ofxVideoWaveTerrainAgent *agent = new ofxVideoWaveTerrainAgent();
        agent->sample_rate = sr;
        agents.push_back(agent);
    }

}

//call from the audio thread
void ofxVideoWaveTerrain::audioOut(float * output, int bufferSize, int nChannels){
    if(nChannels>2){
        cout<<"ofxVideoWaveTerrain error: more than 2 audio channels not supported"<<endl;
        nChannels = 2;
    }
    //trace the agent's path sample by sample and use x and y to index a sinusoidal wave table
    for(int i=0; i<bufferSize; i++){
        mutex.lock();
        elapsed_time += (1./sample_rate); //need enough delay to stay behind latest frame of video
        mutex.unlock();
        double t = elapsed_time - audio_delay;
        if(t<0.) continue;

        if(output){
            for(int c=0; c<nChannels; c++)
                output[i*nChannels+c] = 0;
        }

        double gain = 1./(agents.size()*3.*2.);
        for(int j=0; j<agents.size(); j++){
            ofxVideoWaveTerrainAgent &agent = *agents[j];

            ofFloatColor color = ivv->getColor(agent.p.x, agent.p.y, t);

            agent.update(color, sample_rate, ivv->getAspectRatio());

            if(output){
                for(int c=0; c<nChannels; c++)
                    /*output[i*nChannels+c] +=  sin(6.28318530718*agent.p[c]) *
                        ( agent.color.r*color.r
                        + agent.color.g*color.g
                        + agent.color.b*color.b
                        )*gain*/
                    output[i*nChannels+c] += agent.v[c]*gain;
            }
        }
    }
}

//call from the opengl thread
void ofxVideoWaveTerrain::draw(int x, int y, int w, int h){
    for(int i=0; i<agents.size(); i++)
        agents[i]->draw(x,y,w,h);
}

void ofxVideoWaveTerrain::insert_frame(shared_ptr<ofFloatPixels> frame){
    ivv->insert_frame(frame, elapsed_time);
}

ofxIrregularVideoVolume* ofxVideoWaveTerrain::getVideoVolume(){
    return ivv;
}

double ofxVideoWaveTerrain::getElapsedTime(){
    return elapsed_time;
}
void ofxVideoWaveTerrain::scramble(){
    for(int i=0; i<agents.size(); i++)
        agents[i]->init();
}
void ofxVideoWaveTerrain::setMomentumTime(double x, double time_scale){
    double s = 1;
    for(auto agent:agents){
        agent->setMomentumTime(s*x);
        s*=time_scale;
    }
}
void ofxVideoWaveTerrain::setAudioDelay(double x){
    audio_delay = x;
}
void ofxVideoWaveTerrain::setAgentRate(double x, double rate_scale){
    double s = 1;
    for(auto agent:agents){
        agent->rate = s*x;
        s*=rate_scale;
    }
}
void ofxVideoWaveTerrain::setAgentCombFrequencyP(double x, double rate_scale){
    double s = 1;
    for(auto agent:agents){
        agent->comb_freq_p = s*x;
        s*=rate_scale;
    }
}
void ofxVideoWaveTerrain::setAgentCombFrequencyV(double x, double rate_scale){
    double s = 1;
    for(auto agent:agents){
        agent->comb_freq_v = s*x;
        s*=rate_scale;
    }
}
void ofxVideoWaveTerrain::setAgentCombFeedbackP(double x){
    for(auto agent:agents){
        agent->comb_fb_p = x;
    }
}
void ofxVideoWaveTerrain::setAgentCombFeedbackV(double x){
    for(auto agent:agents){
        agent->comb_fb_v = x;
    }
}
void ofxVideoWaveTerrain::setPathJitter(double x){
    for(auto agent:agents)
        agent->jitter = x;
}

ofxVideoWaveTerrainAgent::ofxVideoWaveTerrainAgent(){
    rate = 0;
    jitter = 0;
    epsilon = 1;
    comb_freq_p = 1;
    comb_freq_v = 1;
    comb_fb_p = 0;
    comb_fb_v = 0;
    color = ofFloatColor(.5, .5, .5, 1.);
    blend_mode = OF_BLENDMODE_DISABLED;
    init();
}
void ofxVideoWaveTerrainAgent::init(){
    p = ofPoint(ofRandom(0,1), ofRandom(0,1), 0);
    v = rate*ofPoint(ofRandom(-1,1), ofRandom(-1,1), 0);
    mutex.lock();
    for(int i=0;i<2;i++){
        history[i] = vector<curve>();
        history[i].push_back(curve());
    }
    cur_hist = 0;
    history[0][0].push_back(p);
    mutex.unlock();
    randomColorWithAlpha(.5);
    randomRotation();
}
void ofxVideoWaveTerrainAgent::draw(int x, int y, int w, int h){
	//draw agent path as line segments

    mutex.lock();
    //swap agent path buffers
    const vector<curve> &hist_to_draw = history[cur_hist];
    ofPoint temp = *(hist_to_draw.rbegin()->rbegin()); //last point of last curve
    cur_hist = 1-cur_hist;
    history[cur_hist].clear();
    curve c;
    c.push_back(temp);
    history[cur_hist].push_back(c);
    mutex.unlock();

    ofPushStyle();
    ofNoFill();
    ofSetColor(color);
    ofEnableBlendMode(blend_mode);
    //ofSetLineWidth(32);
    ofPushMatrix();
    ofScale(w,h);
    ofTranslate(x,y);
    for(int i=0; i<hist_to_draw.size(); i++){
        if(hist_to_draw[i].size()>1){
            ofMesh(OF_PRIMITIVE_LINE_STRIP, hist_to_draw[i]).draw();
            //ofBeginShape();
            //ofVertices(hist_to_draw[i]);
            //ofEndShape();
        }
    }
    ofPopMatrix();
    ofPopStyle();
}

inline void ofxVideoWaveTerrainAgent::update(ofFloatColor terrain_color, double sample_rate, double aspect_ratio){
    this->sample_rate = sample_rate;

    float h;//,s,b;
    //color.getHsb(h,s,b);
    h = terrain_color.getHueAngle()/360.;
    h = (h+rotation)*6.28318530718;

    ofPoint jit;
    if(jitter>0){
        double r = ofRandom(6.28318530718);
        jit = jitter*ofPoint(cos(r), sin(r));
    }

/*    ofPoint new_v = s*b*ofPoint(cos(h),sin(h),0)*rate/sample_rate;
    double eps = 1;
    if(momentum_time>0)
        eps = 1.-pow(2, -1./(sample_rate*momentum_time));
    v += eps*(new_v - v);
*/

    ofPoint new_v;
    ofPoint m(cos(h), sin(h));
    //new_v.x = m.x*v.x - m.y*v.y;
    //new_v.y = m.x*v.y + m.y*v.x;
    new_v = m;

    array<double, 2> h_v;
    array<double, 2> h_p;


    if(comb_fb_p != 0){
        int samps_p = sample_rate / max(comb_freq_p, 1.);
        h_p = p_history.get(samps_p);
        ofPoint toward_old_p = ofPoint(ofWrap(h_p[0]-p.x,-.5,.5), ofWrap(h_p[1]-p.y,-.5,.5));
        toward_old_p /= toward_old_p.length() + .0001;
        //flee old position
        new_v += comb_fb_p*toward_old_p;
    }
    if(comb_fb_v != 0){
        int samps_v = sample_rate / max(comb_freq_v, 1.);
        h_v = v_history.get(samps_v);
        ofPoint old_v = ofPoint(h_v[0], h_v[1]);
        //approach old velocity
        new_v += comb_fb_v*old_v;
    }

    //normalize before momentum
    new_v /= (new_v.length()+.0001);

    //momentum
    v += epsilon*(new_v - v);

    //normalize after momentum
    v /= (v.length()+.0001);

    //
    p += (v*(rate/sample_rate) + jit)*ofPoint(1., aspect_ratio, 0);

    h_v[0] = v.x; h_v[1] = v.y;
    v_history.insert(h_v);

    h_p[0] = p.x; h_p[1] = p.y;
    p_history.insert(h_p);

    ofPoint wrap(0);
    if(p.x>=1) wrap.x = -int(p.x);
    if(p.x<0) wrap.x = int(1-p.x);
    if(p.y>=1) wrap.y = -int(p.y);
    if(p.y<0) wrap.y = int(1-p.y);

    ofPoint pre_wrap = p;
    p+=wrap;

    //cout<< p.x << " " << p.y << endl;

    mutex.lock();
    vector<curve> &hist = history[cur_hist];
    curve &cur_curve = *(hist.rbegin());
    cur_curve.push_back(pre_wrap);
    if(p!=pre_wrap){ //if wrapping around the torus occurred, start a new curve
        curve new_curve;
        ofPoint unwrapped = *(cur_curve.rbegin()) + wrap;
        new_curve.push_back(unwrapped);
        new_curve.push_back(p);
        hist.push_back(new_curve); //start a new segment; note that this invalidates reference cur_curve
    }
    mutex.unlock();
}

void ofxVideoWaveTerrainAgent::randomColorWithAlpha(double alpha){
    color = ofFloatColor(ofRandom(1.), ofRandom(1.), ofRandom(1.), alpha);
    blend_mode = OF_BLENDMODE_ALPHA;
}

void ofxVideoWaveTerrainAgent::randomRotation(){
    rotation = ofRandom(1.);
}

/*
void ofxVideoWaveTerrainAgent::setJitter(double x){
    mutex.lock();
    jitter = x;
    mutex.unlock();
}
void ofxVideoWaveTerrainAgent::setRate(double x){
    mutex.lock();
    rate = x;
    mutex.unlock();
}
void ofxVideoWaveTerrainAgent::setCombFrequency(double x){
    mutex.lock();
    comb_freq = x;
    mutex.unlock();
}
*/
void ofxVideoWaveTerrainAgent::setMomentumTime(double momentum_time){
    mutex.lock();
    epsilon = 1;
    if(momentum_time>0)
        epsilon = 1.-pow(2, -1./(sample_rate*momentum_time*.001));
    mutex.unlock();
}

ofxIrregularVideoVolume::ofxIrregularVideoVolume(int ftk, double ar){
    frames_to_keep = ftk;
    aspect_ratio = ar;
}

void ofxIrregularVideoVolume::insert_frame(shared_ptr<ofFloatPixels> frame, double t){
    //insert a new frame to the irregularly sampled video volume
    //frames must be regularly sampled ofPixels objects but can be any resolution
    //and placed irregularly in time

    //cout<<"agent coords: ("<<agent_coords.x<<", "<<agent_coords.y<<")"<<endl;

    if(!frame->isAllocated() || frame->getWidth() <= 0 || frame->getHeight() <= 0){
        cout<<"ofxVideoWaveTerrain warning: attempt insert frame with bad size "
            <<frame->getWidth()<<" by "<<frame->getHeight()<<endl;
            return;
    }

    mutex.lock();
    //insert with hint that this frame goes at the end
    frames.insert(frames.end(), pair<double, shared_ptr<ofFloatPixels> >(t,frame));
    if(frames.size()>frames_to_keep){
        //delete frames.begin()->second;
        frames.erase(frames.begin());
    }
    mutex.unlock();
    //cout<< "earliest: "<<frames.begin()->first<<", latest: "<<frames.end()->first<<endl;
}

ofFloatColor ofxIrregularVideoVolume::getColor(double x, double y, double t){
    //trilinear interpolation of the video volume at x,y,t
    // x,y in normalized coordinates (0-1), t in seconds
    mutex.lock();
     if(frames.size()<2){
        cout<<"ofxVideoWaveTerrain warning: fewer than 2 frames available"<<endl;
        mutex.unlock();
        return ofFloatColor();
    }
    map<double,shared_ptr<ofFloatPixels> >::iterator it;
    double t_earliest = frames.begin()->first;
    double t_latest = frames.rbegin()->first;
    if(t>t_latest){
        //cout<< "ofxVideoWaveTerrain warning: time "<<t<<" is after latest frame at "<< t_latest <<endl;
        it = frames.end();
        it--;
    }
    else{
        if(t<t_earliest)
            if(t>=0) cout<< "ofxVideoWaveTerrain warning: time "<<t<<" is before earliest frame at "<< t_earliest <<endl;
        it = frames.lower_bound(t);
    }
    double t_after = it->first;
    ofFloatPixels &after = *(it->second);
    if(it!=frames.begin())
        it--;
    double t_before = it->first;
    ofFloatPixels &before = *(it->second);

    double mt =0;
    double denom = t_after - t_before;
    if(denom>0)
        mt = (t - t_before)/denom;
    int bw = before.getWidth();
    int bh = before.getHeight();
    int lxb = int(x*bw)%bw;
    int lyb = int(y*bh)%bh;
    double mxb = x*bw - int(x*bw);
    double myb = y*bh - int(y*bh);
    int uxb = int(1+x*bw)%bw;
    int uyb = int(1+y*bh)%bh;
    int aw = after.getWidth();
    int ah = after.getHeight();
    int lxa = int(x*aw)%aw;
    int lya = int(y*ah)%ah;
    int uxa = int(1+x*aw)%aw;
    int uya = int(1+y*ah)%ah;
    double mxa = x*aw - int(x*aw);
    double mya = y*ah - int(y*ah);
    ofFloatColor before_lower = before.getColor(lxb, lyb).getLerped(before.getColor(uxb, lyb), mxb);
    ofFloatColor before_upper = before.getColor(lxb, uyb).getLerped(before.getColor(uxb, uyb), mxb);
    ofFloatColor after_lower = after.getColor(lxa, lya).getLerped(after.getColor(uxa, lya), mxa);
    ofFloatColor after_upper = after.getColor(lxa, uya).getLerped(after.getColor(uxa, uya), mxa);
    mutex.unlock();
    ofFloatColor before_ = before_lower.getLerped(before_upper, myb);
    ofFloatColor after_ = after_lower.getLerped(after_upper, mya);
    return before_.getLerped(after_, mt);
}
int ofxIrregularVideoVolume::getFramesToKeep(){
    return frames_to_keep;
}
double ofxIrregularVideoVolume::getAspectRatio(){
    return aspect_ratio;
}
void ofxIrregularVideoVolume::setFramesToKeep(int x){
    mutex.lock();
    frames_to_keep = x;
    while(frames.size()>frames_to_keep)
        frames.erase(frames.begin());
    mutex.unlock();
}
void ofxIrregularVideoVolume::setAspectRatio(double x){
    mutex.lock();
    aspect_ratio = x;
    mutex.unlock();
}
