#include "ofxVideoWaveTerrain.h"

ofxVideoWaveTerrain::ofxVideoWaveTerrain(int ftk=100, double ttk=10, int sr=44100, double del=.2){
	frames_to_keep = ftk;
	time_to_keep = ttk;
	agent_rate = 440.;
    agent_acc = 1.;
	sample_rate = sr;
	elapsed_time = 0;
	audio_delay = del;
    //vertex_skip = 8;

	for(int i=0;i<2;i++){
        agent_hist[i] = vector<vector<ofPoint> >();
        agent_hist[i].push_back(vector<ofPoint>());
    }
    cur_hist = 0;
}

//call from the audio thread
void ofxVideoWaveTerrain::audioOut(float * output, int bufferSize, int nChannels){
    //trace the agent's path sample by sample and use x and y to index a sinusoidal wave table
    for(int i=0; i<bufferSize; i++){
        mutex.lock();
        elapsed_time += (1./sample_rate); //need enough delay to stay behind latest frame of video
        double t = elapsed_time - audio_delay;
        if(t<0.){
            mutex.unlock();
            continue;
        }

        ofFloatColor color = getColor(agent_coords.x, agent_coords.y, t);
        mutex.unlock();

        float h,s,b;
        color.getHsb(h,s,b);
        h*=6.28318530718;
        //ofs = 2.*ofPoint(color.r-.5, color.g-.5, 0);
        double eps = 1.-pow(.5, 3000./sample_rate);
        agent_vel += eps*(s*ofPoint(cos(h),sin(h),0) - agent_vel);
        agent_coords += agent_vel*agent_rate/sample_rate;

        ofPoint pre_wrap = agent_coords;
        agent_coords.x = ofWrap(agent_coords.x, 0, 1);
        agent_coords.y = ofWrap(agent_coords.y, 0, 1);

        ofPoint jit;
        mutex.lock();
        //if(!(i%vertex_skip) || agent_coords!=pre_wrap)
        jit = .003*ofPoint(ofRandom(-1,1), ofRandom(-1,1));
        agent_hist[cur_hist].rbegin()->push_back(pre_wrap+jit);
        if(agent_coords!=pre_wrap){
            agent_hist[cur_hist].push_back(vector<ofPoint>()); //start a new segment
            agent_hist[cur_hist].rbegin()->push_back(agent_coords+jit); //push a new point onto the latest segment
        }
        mutex.unlock();
        for(int c=0; c<nChannels; c++)
            output[i*nChannels+c] = sin(6.28318530718*agent_coords[c]);
    }
}

//call from the opengl thread
void ofxVideoWaveTerrain::draw(int x, int y, int w, int h){
	//draw agent path as line segments

    mutex.lock();
    //swap agent path buffers
    const vector<vector<ofPoint> > &hist = agent_hist[cur_hist];
    cur_hist = 1-cur_hist;
    agent_hist[cur_hist].clear();
    agent_hist[cur_hist].push_back(vector<ofPoint>());
    mutex.unlock();

    ofPushStyle();
    ofNoFill();
    ofSetColor(ofFloatColor(1,1,1,1));
    ofPushMatrix();
    ofScale(w,h);//ofScale(1./w,1./h);
    ofTranslate(x,y);
    for(int i=0; i<hist.size(); i++){
        if(hist[i].size()>1){
            ofBeginShape();
            ofVertices(hist[i]);
            ofEndShape();
        }
    }
    ofPopMatrix();
    ofPopStyle();
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
    int lxb = int(x*bw);
    int lyb = int(y*bh);
    int uxb = int(1+x*bw)%bw;
    int uyb = int(1+y*bh)%bh;
    int aw = after.getWidth();
    int ah = after.getHeight();
    int lxa = int(x*aw);
    int lya = int(y*ah);
    int uxa = int(1+x*aw)%aw;
    int uya = int(1+y*ah)%ah;
    ofFloatColor before_lower = before.getColor(lxb, lyb).getLerped(before.getColor(uxb, lyb), x);
    ofFloatColor before_upper = before.getColor(lxb, uyb).getLerped(before.getColor(uxb, uyb), x);
    ofFloatColor after_lower = after.getColor(lxa, lya).getLerped(after.getColor(uxa, lya), x);
    ofFloatColor after_upper = after.getColor(lxa, uya).getLerped(after.getColor(uxa, uya), x);
    ofFloatColor before_ = before_lower.getLerped(before_upper, y);
    ofFloatColor after_ = after_lower.getLerped(after_upper, y);
    return before_.getLerped(after_, mt);
}
double ofxVideoWaveTerrain::getElapsedTime(){
    return elapsed_time;
}