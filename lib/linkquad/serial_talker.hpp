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
#include <termios.h>
#include <unistd.h>
#include <map>
#include "serial_base.hpp"

namespace LinkQuad {
    namespace comm {
        namespace serial {
#include "link_api/packet.h"
            typedef std::map<int, pthread_mutex_t*> mutexmap;
            extern mutexmap talker_mutexmap;
            template<typename T>
            class serial_talker : serial_base {
                private:
                    int fd;
                    T message;
                    pthread_t thread_id;

                    uint8_t snd_buffer[sizeof(T) + 10];

                    void lock_port() {
                        pthread_mutex_t* m = talker_mutexmap[fd];
                        if(m == NULL) {
                            m = new pthread_mutex_t;
                            pthread_mutex_init(m, NULL);
                            talker_mutexmap[fd] = m;
                        }

                        pthread_mutex_lock(m);
                    }

                    void unlock_port() {
                        pthread_mutex_unlock(talker_mutexmap[fd]);
                    }

                    void send_message() {
                        unsigned char length = 0;
                        packet_create_header(&message.metadata, snd_buffer, &length);
                        length += message.marshal(&snd_buffer[length]);
                        packet_seal(snd_buffer, &length);

                        if (write(fd, snd_buffer, length) != length)
                        {
                            std::cerr << "write() failed" << std::endl;
                        }
                    }

                public:

                    void init_thread() {
                        lock_port();
                        send_message();
                        unlock_port();
                    }

                    serial_talker(const T msg, const int fd_)
                        :   fd(fd_),
                            message(msg)
                    {
                        pthread_create(&thread_id, NULL, thread_initer<serial_talker<T> >, this);
                    }
            };
        }
    }
}

#endif
