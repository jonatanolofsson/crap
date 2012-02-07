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


#ifndef CRAP_LINKQUAD_SERIAL_COMMUNICATION_TALKER
#define CRAP_LINKQUAD_SERIAL_COMMUNICATION_TALKER

#include <pthread.h>    // POSIX threads
#include <semaphore.h>

#include <termios.h>
#include <unistd.h>
#include <map>
#include "serial_base.hpp"
#include <deque>

#include <iostream>

namespace LinkQuad {
    namespace comm {
        namespace serial {
#include "link_api/packet.h"

            typedef std::map<int, pthread_mutex_t*> mutexmap;

            class serial_talker {
                private:
                    typedef std::pair<uint8_t*, unsigned int> message_descriptor;
                    typedef std::deque<message_description> message_queue;
                    int fd;
                    T message;
                    pthread_t thread_id;
                    bool quit_thread;
                    sem_t messages_available;
                    pthread_mutex_t* messages_lock;

                    void send_loop() {
                        message_queue::iterator message;
                        while(!quit_thread) {
                            sem_wait(messages_available);
                            message = messages.begin();
                            if (write(fd, message->first, message->second) != message->second)
                            {
                                std::cerr << "write() failed" << std::endl;
                            }
                            pthread_mutex_lock(messages_lock);
                            messages.pop_front();
                            pthread_mutex_unlock(messages_lock);
                        }
                    }

                public:

                    void init_thread() {
                        send_loop();
                    }

                    template<typename T>
                    void send(const T message) {
                        unsigned char length = 0;
                        uint8_t* buffer = malloc(sizeof(T) + 10);

                        packet_create_header(&message.metadata, buffer, &length);
                        length += message.marshal(&buffer[length]);
                        packet_seal(buffer, &length);
                        pthread_mutex_lock(messages_lock);
                        messages.push_back(message_descriptor(mem, length));
                        pthread_mutex_unlock(messages_lock);
                        sem_post(messages_available);
                    }

                    serial_talker(const int fd_)
                        :   fd(fd_)
                    {
                        pthread_mutex_init(messages_lock, NULL);
                        sem_init(got_messages, 0, 0);
                        pthread_create(&thread_id, NULL, thread_initer<serial_talker<T> >, this);
                    }

                    ~serial_talker() {
                        quit_thread();
                        pthread_join(thread_id, NULL);
                    }
            };
        }
    }
}

#endif
