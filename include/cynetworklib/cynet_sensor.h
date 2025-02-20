#ifndef CYNET_SENSOR_H
#define CYNET_SENSOR_H

#include <vector>
#include <map>
#include <algorithm>
#include <typeinfo>
#include <iostream>
#include <mutex>
#include <atomic>

class CySensor;

class CySensorMeasurementProcessor{

protected:
    CySensor *s;

public:
    CySensorMeasurementProcessor(CySensor *sensor){s = sensor;}
    virtual ~CySensorMeasurementProcessor(){};

    virtual void add_measurement(uint16_t in) = 0;

    friend class CySensor;
};

class CySensor{

    std::vector<CySensorMeasurementProcessor *> proc;

    std::atomic<std::uint16_t> measurement;

    int sensor_name;	/* 0: Temperature, N: TN.  Note that N could be different from sensor's index because of cut sensors */

    std::mutex mutex;

   // CyStats *stats; /* Contains the stats of a sensor. If null, this functionality is disabled */
public:

    CySensor(): measurement(0), sensor_name(-1) {}
    CySensor(const CySensor &s){ 
        sensor_name = s.sensor_name;
        measurement = 0;
        proc = s.proc;
    }

    CySensor& operator=(const CySensor& s){
        sensor_name = s.sensor_name;
        measurement = 0;
        proc = s.proc;
        return *this;
    }

    int get_name(){ return sensor_name; }

    void attach_processor(CySensorMeasurementProcessor *p) {
        mutex.lock();
        p->s = this;
        proc.push_back(p);
        mutex.unlock();
    }
    void detach_processor(CySensorMeasurementProcessor *p) {
        mutex.lock();
        auto it = std::remove(proc.begin(), proc.end(), p);
        if (it != proc.end()) {
          if (*it) delete *it;
          proc.erase(it, proc.end());
        }
        mutex.unlock();
    }

    void clear_processors() {
        mutex.lock();
        while (proc.size()) {
          proc.erase(std::remove(proc.begin(), proc.end(), *proc.begin()),
                     proc.end());
        }
        mutex.unlock();
      }

    template<typename T> T* get_processor() const {
        auto it = std::find_if(proc.begin(),proc.end(), [](const CySensorMeasurementProcessor* p) { return typeid(*p) == typeid(T) ;});
        return it == proc.end() ? 0 : (T*)*it;
    };

    void add_measurement(uint16_t in) {
        measurement = in;
        mutex.lock();
        for (auto &i : proc) {
          i->add_measurement(in);
        }
        mutex.unlock();
    } 
    
    uint16_t get_measurement() const {return measurement;};

    friend class EcatHandler;
};


class CySensorActivity : public CySensorMeasurementProcessor {

    bool working;		/* whether sensor was active at any time in the past */
    bool active;		/* whether the sensor is currently active */
    uint16_t baseline;
    uint16_t threshold;
    uint16_t persistent_activation;  /* used to record if the sensor was active in the previous pers_samples samples*/
    uint16_t pers_samples;

public: 

    CySensorActivity(CySensor *sensor=0,uint16_t th=333, uint16_t ps=100):CySensorMeasurementProcessor(sensor),
                                                                        working(false),
                                                                        active(false),
                                                                        baseline(-1),
                                                                        threshold(th),
                                                                        persistent_activation(0),
                                                                        pers_samples(ps){}
    CySensorActivity(uint16_t ps): CySensorActivity(0,333,ps){};

    virtual void add_measurement(uint16_t in){
        
        
        if(in < baseline)
            baseline = in;

        active = in > (baseline + threshold) && baseline != 0 && baseline != 0xFFFF;
        
        if(persistent_activation)
            persistent_activation--;

        if(active){
            working = true;
            persistent_activation = pers_samples;
        }
        else{
            if(in - baseline > threshold/2)
                baseline = in;
        }
    }

    bool has_recent_activity(){return persistent_activation>0;}
    bool is_active(){return active;}
    bool is_working(){return working;}
};
 
#endif
