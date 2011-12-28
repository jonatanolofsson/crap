/**
 * Copyright 2011, 2012 Jonatan Olofsson
 * 
 * This file is part of C++ Robot Automation Platform (CRAP).
 * 
 * CRAP is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * CRAP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with CRAP.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#ifndef CRAP_COMMUNICATION_HPP_
#define CRAP_COMMUNICATION_HPP_

#include <string>
#include "crap/types.hpp"
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

namespace CRAP {
    namespace comm {
        //~ template<typename T>
        //~ void latch_message(const std::string, const T* , bool = false);
        template<typename T>
        void send(const std::string, const T&, const bool = false);
        template<typename T>
        void listen(void(*)(const std::string, const T&));
        
        extern listener_map_t listeners;
        extern boost::mutex listener_lock;
        extern latched_messages_map_t latched_messages;
        
        typedef boost::interprocess::scoped_lock<boost::mutex> scoped_lock;
        
        template<typename T>
        void send(const std::string topic, const T& msg, const bool latch) {
            listener_map_t::iterator topic_listeners;
            
            if(latch) {
                latched_messages_map_t::iterator lmsg = latched_messages.find(topic);
                if(lmsg != latched_messages.end()) {
                    delete (T*)(lmsg->second);
                }
                latched_messages[topic] = (void*) new T(msg);
            }
            
            // Enforce message order consistency __in topic__
            // while allowing multiple topics to execute at once
            static boost::mutex send_lock;
            scoped_lock l(send_lock);
            
            {
                scoped_lock ll(listener_lock);
                topic_listeners = listeners.find(topic);
                if(topic_listeners == listeners.end()) {
                    // Screw this, no-one's listening anyway..
                    return;
                }
            }
            
            for(listeners_t::iterator fn = topic_listeners->second.begin(); fn != topic_listeners->second.end(); ++fn) {
                boost::thread(boost::bind((void(*)(const T&))(*fn), msg));
            }
        }
        
        template<typename T>
        void listen(const std::string topic, void(*fn)(const T&)) {
            scoped_lock l(listener_lock);
            listener_map_t::iterator listener_list = listeners.find(topic);
            if(listener_list == listeners.end()) {
                listener_list = listeners.insert(listener_map_t::value_type(topic, listeners_t())).first;
            }
            listener_list->second.push_back((listener_t)fn);
            
            latched_messages_map_t::iterator latched_message = latched_messages.find(topic);
            if(latched_message != latched_messages.end()) {
                boost::thread(boost::bind((void(*)(const T&))(*fn), *(T*)(latched_message->second)));
            }
        }
    }
}

//CRAP_COMMUNICATION_HPP_
#endif