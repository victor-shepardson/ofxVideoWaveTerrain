#include "ofxVideoWaveTerrain.h"

ofxVideoWaveTerrain::ofxVideoWaveTerrain(int ftk=100, double ttk=10, int sr=44100, double del=.2){
	frames_to_keep = ftk;
	time_to_keep = ttk;
	base_agent_rate = 30.;
	sample_rate = sr;
	elapsed_time = 0;
	audio_delay = del;
    aspect_ratio = 1.;

	double momentum_time = 0;
	double path_jitter = 0;

    agents.push_back(ofxVideoWaveTerrainAgent(base_agent_rate, path_jitter, momentum_time, ofFloatColor(0,1,0)));
    agents.push_back(ofxVideoWaveTerrainAgent(pow(1.5,1)*base_agent_rate, path_jitter, momentum_time, ofFloatColor(0,0,1)));
    agents.push_back(ofxVideoWaveTerrainAgent(pow(1.5,2)*base_agent_rate, path_jitter, momentum_time, ofFloatColor(1,0,0)));
    agents.push_back(ofxVideoWaveTerrainAgent(pow(1.5,3)*base_agent_rate, path_jitter, momentum_time, ofFloatColor(.7,0,.7)));
    agents.push_back(ofxVideoWaveTerrainAgent(pow(1.5,4)*base_agent_rate, path_jitter, momentum_time, ofFloatColor(.7,.7,0)));
    agents.push_back(ofxVideoWaveTerrainAgent(pow(1.5,5)*base_agent_rate, path_jitter, momentum_time, ofFloatColor(0,.7,.7)));
}

//call from the audio thread
void ofxVideoWaveTerrain::audioOut(float * output, int bufferSize, int nChannels){
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

        const int n_agents = agents.size();
        for(int j=0; j<agents.size(); j++){
            ofxVideoWaveTerrainAgent &agent = agents[j];

            mutex.lock();
            ofFloatColor color = getColor(agent.p.x, agent.p.y, t);
            mutex.unlock();

            agent.update(mutex, color, sample_rate, aspect_ratio);

            if(nChannels>2){
                cout<<"ofxVideoWaveTerrain error: more than 2 audio channels not supported"<<endl;
                nChannels = 2;
            }
            if(output){
                for(int c=0; c<nChannels; c++)
                    output[i*nChannels+c] += sin(6.28318530718*agent.p[c])*(1./n_agents);
            }
        }
    }
}

//call from the opengl thread
void ofxVideoWaveTerrain::draw(int x, int y, int w, int h){
    for(int i=0; i<agents.size(); i++)
        agents[i].draw(mutex,x,y,w,h);
}

void ofxVideoWaveTerrain::insert_frame(ofFloatPixels &frame, double t){
    //insert a new frame to the irregularly sampled video volume
    //frames must be regularly sampled ofPixels objects but can be any resolution
    //and placed irregularly in time

    //cout<<"agent coords: ("<<agent_coords.x<<", "<<agent_coords.y<<")"<<endl;

    mutex.lock();
    //insert with hint that this frame goes at the end
    frames.insert(frames.end(), pair<double, ofFloatPixels>(t,frame));
    if(frames.size()>frames_to_keep || frames.begin()->first < t - time_to_keep)
        frames.erase(frames.begin());
    mutex.unlock();
    //cout<< "earliest: "<<frames.begin()->first<<", latest: "<<frames.end()->first<<endl;
}
void ofxVideoWaveTerrain::insert_frame(ofFloatPixels &frame){
    insert_frame(frame, elapsed_time);
}

ofFloatColor ofxVideoWaveTerrain::getColor(double x, double y, double t){
    //trilinear interpolation of the video volume at x,y,t
    // x,y in normalized coordinates (0-1), t in seconds

    if(frames.size()<2){
        cout<<"ofxVideoWaveTerrain warning: fewer than 2 frames available"<<endl;
        return ofFloatColor();
    }
    double t_earliest = frames.begin()->first;
    if(t<t_earliest){
        if(t>=0) cout<< "ofxVideoWaveTerrain warning: time "<<t<<" is before earliest frame at "<< t_earliest <<endl;
        return ofFloatColor();
    }
    double t_latest = frames.rbegin()->first;
    if(t>=t_latest){
        cout<< "ofxVideoWaveTerrain warning: time "<<t<<" is after latest frame at "<< t_latest <<endl;
        return ofFloatColor();
    }
    map<double,ofFloatPixels>::iterator it = frames.upper_bound(t);
    double t_after = it->first;
    ofFloatPixels &after = it->second;
    double t_before = (--it)->first;
    ofFloatPixels &before = it->second;

    //could do the below outside of mutex lock, if we trust that frames_to_keep and
    //time_to_keep are large enough not to delete the current frames

    //more conservatively, could separate the reads and interpolation as much as possible

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
    ofFloatColor before_ = before_lower.getLerped(before_upper, myb);
    ofFloatColor after_ = after_lower.getLerped(after_upper, mya);
    return before_.getLerped(after_, mt);
}
double ofxVideoWaveTerrain::getElapsedTime(){
    return elapsed_time;
}
void ofxVideoWaveTerrain::scramble(){
    mutex.lock();
    for(int i=0; i<agents.size(); i++)
        agents[i].init();
    mutex.unlock();
}
void ofxVideoWaveTerrain::setMomentumTime(double x){
    mutex.lock();
    for(int j=0; j<agents.size(); j++)
        agents[j].momentum_time = x;
    mutex.unlock();
}
void ofxVideoWaveTerrain::setAudioDelay(double x){
    mutex.lock();
    audio_delay = x;
    mutex.unlock();
}
void ofxVideoWaveTerrain::setAgentRate(double x){
    mutex.lock();
    base_agent_rate = x;
    for(int j=0; j<agents.size(); j++)
        agents[j].rate = pow(2,j)*x;
    mutex.unlock();
}
void ofxVideoWaveTerrain::setPathJitter(double x){
    mutex.lock();
    for(int j=0; j<agents.size(); j++)
        agents[j].jitter = x;
    mutex.unlock();
}
void ofxVideoWaveTerrain::setFramesToKeep(int x){
    mutex.lock();
    frames_to_keep = x;
    while(frames.size()>frames_to_keep)
        frames.erase(frames.begin());
    mutex.unlock();
}
void ofxVideoWaveTerrain::setTimeToKeep(double x){
    mutex.lock();
    time_to_keep = x;
    while(frames.size()>0 && frames.begin()->first < getElapsedTime() - time_to_keep)
        frames.erase(frames.begin());
    mutex.unlock();
}
void ofxVideoWaveTerrain::setAspectRatio(double x){
    mutex.lock();
    aspect_ratio = x;
    //cout<<aspect_ratio<<endl;
    mutex.unlock();
}

ofxVideoWaveTerrainAgent::ofxVideoWaveTerrainAgent(double r, double j, double mt, ofFloatColor c = ofFloatColor(1,1,1)){
    rate = r;
    jitter = j;
    momentum_time = mt;
    color=c;
    init();
}
void ofxVideoWaveTerrainAgent::init(){
    p = ofPoint(ofRandom(0,1), ofRandom(0,1), 0);
    v = ofPoint(0,0,0);
    for(int i=0;i<2;i++){
        history[i] = vector<curve>();
        history[i].push_back(curve());
    }
    cur_hist = 0;
    history[0][0].push_back(p);
}
void ofxVideoWaveTerrainAgent::draw(ofMutex &mutex, int x, int y, int w, int h){
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
    ofEnableBlendMode(OF_BLENDMODE_ADD);
    ofPushMatrix();
    ofScale(w,h);
    ofTranslate(x,y);
    for(int i=0; i<hist_to_draw.size(); i++){
        if(hist_to_draw[i].size()>1){
            ofBeginShape();
            ofVertices(hist_to_draw[i]);
            ofEndShape();
        }
    }
    ofPopMatrix();
    ofPopStyle();
}

inline void ofxVideoWaveTerrainAgent::update(ofMutex &mutex, ofFloatColor color, double sample_rate, double aspect_ratio){
    float h,s,b;
    color.getHsb(h,s,b);
    h*=6.28318530718;

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
    p += ofPoint(1., aspect_ratio, 0)*v + jit;
*/

    ofPoint new_v = ofPoint(cos(h),sin(h),0)*rate/sample_rate;
    double eps = 1;
    double mt = momentum_time*(1-s);
    if(mt>0)
        eps = 1.-pow(2, -1./(sample_rate*mt));
    v += eps*(new_v - v);
    p += ofPoint(1., aspect_ratio, 0)*v + jit;

    ofPoint wrap;
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
    ofPoint unwrapped = *(cur_curve.rbegin()) + wrap;
    cur_curve.push_back(pre_wrap);
    if(p!=pre_wrap){
        curve new_curve;
        new_curve.push_back(unwrapped);
        new_curve.push_back(p);
        hist.push_back(new_curve); //start a new segment; note that this invalidates reference cur_curve
    }
    mutex.unlock();
}
