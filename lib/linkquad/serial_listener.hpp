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


#ifndef CRAP_LINKQUAD_SERIAL_COMMUNICATION_LISTENER
#define CRAP_LINKQUAD_SERIAL_COMMUNICATION_LISTENER


#include <pthread.h>    // POSIX threads
#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include "serial_base.hpp"

namespace LinkQuad {
    namespace comm {
        namespace serial {

#include "link_api/packet.h"
            const int RCVBUFSIZE    = 1024;
            template<typename T>
            class serial_listener : public serial_base {
                private:
                    int fd;
                    T datacontainer;
                    pthread_t thread_id;
                    void(*callback)(const T);
                    bool quit_thread;

                    uint8_t rcv_buffer[sizeof(T) + 10];

                    struct {
                        size_t length;
                        size_t offset;
                        void(serial_listener::*cb)(const uint8_t*, const size_t);
                    } reading;

                    void read(unsigned int length, void(serial_listener::*cb)(const uint8_t*, const size_t)) {
                        reading.length = length;
                        reading.cb = cb;
                    }

                    void loop_begin();

                    void read_identifier_1(const uint8_t* msg, const size_t) {
                        if(msg[0] == PACKET_IDENTIFIER_1) {
                            read(1, &serial_listener<T>::read_identifier_2);
                        } else loop_begin();
                    }

                    void read_identifier_2(const uint8_t* msg, const size_t) {
                        if(msg[0] == PACKET_IDENTIFIER_2) {
                            read(1, &serial_listener<T>::read_packet_length);
                        } else loop_begin();
                    }

                    void read_packet_length(const uint8_t* msg, const size_t) {
                        if(msg[0] > 1) { // Must have enough data for at least version number and checksum
                            read(msg[0], &serial_listener<T>::read_packet_data);
                        } else loop_begin();
                    }

                    void read_packet_data(const uint8_t* msg, const size_t size) {
                        if(packet_flat_checksum(const_cast<uint8_t*>(msg), size-1) != msg[size-1]) {
                            std::cerr << "Checksum error" << std::endl;
                        } else {
                            unsigned char data_offset, data_length;
                            packet_parse_header(&rcv_buffer[PACKET_INDEX_LENGTH], &datacontainer.metadata, &data_offset, &data_length);
                            if(datacontainer.set(&rcv_buffer[data_offset], data_length)) {
                                if(callback) callback(datacontainer);
                            } else {
                                std::cerr << "Data size mismatch" << std::endl;
                            }
                        }
                        loop_begin();
                    }

                    void read_loop() {
                        while(!quit_thread) {
                            int serial_recv_byte_count = 0;
                            size_t remaining = reading.length;
                            while(remaining > 0) {
                                if (( serial_recv_byte_count  = ::read( fd, &rcv_buffer[reading.offset], remaining )) < 0 ) {
                                    std::cerr << "read() failed" << std::endl;
                                    usleep(100);
                                    loop_begin();
                                    remaining = reading.length;
                                } else {
                                    remaining -= serial_recv_byte_count;
                                    reading.offset += serial_recv_byte_count;
                                }
                            }

                            ((this)->*(reading.cb))(&rcv_buffer[reading.offset-reading.length], reading.length);
                        }
                    }


                public:

                    void init_thread() {
                        loop_begin();
                        read_loop();
                    }
                    serial_listener(void(*cb_)(const T), const int fd_ = 0)
                        :   fd(fd_),
                            callback(cb_),
                            quit_thread(false)

                    {
                        pthread_create(&thread_id, NULL, thread_initer<serial_listener<T> >, this);
                    }

                    ~serial_listener() {
                        void* result;
                        quit_thread = true;
                        pthread_join(thread_id, &result);
                    }
            };
            template<typename T>
            inline void serial_listener<T>::loop_begin() {
                reading.offset = 0;
                read(1, &serial_listener<T>::read_identifier_1);
            }
        }
    }
}

#endif
