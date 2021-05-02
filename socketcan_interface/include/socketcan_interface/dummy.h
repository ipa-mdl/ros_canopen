#ifndef SOCKETCAN_INTERFACE_DUMMY_H
#define SOCKETCAN_INTERFACE_DUMMY_H

#include <deque>
#include <unordered_map>

#include "interface.h"
#include "dispatcher.h"
#include "string.h"
#include "logging.h"
#include <boost/thread/thread.hpp>
#include <boost/algorithm/string.hpp>


namespace can{

class DummyInterface : public DriverInterface{
    using FrameDispatcher = FilteredDispatcher<unsigned int, CommInterface::FrameListener>;
    using StateDispatcher = SimpleDispatcher<StateInterface::StateListener>;
    using Map = std::unordered_map<std::string, Frame>;
    FrameDispatcher frame_dispatcher_;
    StateDispatcher state_dispatcher_;
    State state_;
    Map map_;
    std::deque<can::Frame> out_;
    bool loopback_;
    bool trace_;
    boost::mutex mutex_;
    boost::condition_variable cond_;

    bool add_noconv(const std::string &k, const Frame &v, bool multi){
        boost::mutex::scoped_lock cond_lock(mutex_);
        if(multi || map_.find(k) == map_.end()){
              map_.insert( std::make_pair(boost::to_lower_copy(k), v));
              return true;
        }
        return false;
    }
    void setDriverState(State::DriverState state){
        boost::mutex::scoped_lock lock(mutex_);
        if(state_.driver_state != state){
            state_.driver_state = state;
            state_dispatcher_.dispatch(state_);
        }
        cond_.notify_all();
    }
public:
    DummyInterface() : loopback_(false), trace_(false) {}
    DummyInterface(bool loopback) : loopback_(loopback), trace_(false) {}
    virtual ~DummyInterface() { shutdown(); }


    bool add(const std::string &k, const Frame &v, bool multi){
        return add_noconv(boost::to_lower_copy(k), v, multi);
    }
    bool add(const Frame &k, const Frame &v, bool multi){
        return add_noconv(tostring(k,true), v, multi);
    }
    bool add(const std::string &k, const std::string &v, bool multi){
        return add(k, toframe(v), multi);
    }
    bool add(const Frame &k, const std::string &v, bool multi){
        return add(k, toframe(v), multi);
    }
    virtual bool send(const Frame & msg){
        boost::mutex::scoped_lock cond_lock(mutex_);
        if (trace_) {
            ROSCANOPEN_DEBUG("socketcan_interface", "send: " << msg);
        }
        out_.push_back(msg);
        cond_lock.unlock();
        cond_.notify_all();
        return true;
    }

    virtual FrameListenerConstSharedPtr createMsgListener(const FrameFunc &delegate){
        return frame_dispatcher_.createListener(delegate);
    }
    virtual FrameListenerConstSharedPtr createMsgListener(const Frame::Header&h , const FrameFunc &delegate){
        return frame_dispatcher_.createListener(h.key(), delegate);
    }

    // methods from StateInterface
    virtual bool recover(){return false;};

    virtual State getState(){
        boost::mutex::scoped_lock cond_lock(mutex_);
        return state_;
    }

    virtual void shutdown(){
        flush();
        setDriverState(State::closed);
    };

    virtual bool translateError(unsigned int internal_error, std::string & str){
        if (!internal_error) {
            str = "OK";
            return true;
        }
        return false;
    };

    virtual bool doesLoopBack() const {
        return loopback_;
    };

    void flush(){
        while (true) {
            {
                boost::mutex::scoped_lock cond_lock(mutex_);
                if (out_.empty()) {
                    return;
                }            
            }
            boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
        }
    }

    virtual void run(){
        boost::mutex::scoped_lock cond_lock(mutex_);
        while (true) {

            state_.driver_state = State::ready;
            state_dispatcher_.dispatch(state_);

            cond_.wait(cond_lock);
            while(!out_.empty()){
                const can::Frame msg = out_.front();
                out_.pop_front();
                if(loopback_){
                    frame_dispatcher_.dispatch(msg.key(), msg);
                    if (trace_) {
                        ROSCANOPEN_DEBUG("socketcan_interface", "receive: " << msg);
                    }
                }
                try{
                    std::pair <Map::iterator, Map::iterator> r = map_.equal_range(tostring(msg, true));
                    for (Map::iterator it=r.first; it!=r.second; ++it){
                        if (trace_) {
                            ROSCANOPEN_DEBUG("socketcan_interface", "receive: " << it->second);
                        }
                        frame_dispatcher_.dispatch(it->second.key(), it->second);
                    }
                }
                catch(const std::out_of_range &e){
                }
            }
            if (state_.driver_state == State::closed) {
                return;
            }
        }
    }

    bool init(const std::string &device, bool loopback){
        loopback_ = loopback;
        setDriverState(State::open);
        return true;
    }

    virtual bool init(const std::string &device, bool loopback, SettingsConstSharedPtr settings) {
        if(DummyInterface::init(device, loopback)) {
            trace_ = settings->get_optional("trace", false);
            return true;
        } else {
            return false;
        }
    }
    virtual StateListenerConstSharedPtr createStateListener(const StateFunc &delegate){
      return state_dispatcher_.createListener(delegate);
    }

};
using DummyInterfaceSharedPtr = std::shared_ptr<DummyInterface>;
template <typename T> class ThreadedInterface;
using ThreadedDummyInterface = ThreadedInterface<DummyInterface>;
using ThreadedDummyInterfaceSharedPtr = std::shared_ptr<ThreadedDummyInterface>;

}

#endif
