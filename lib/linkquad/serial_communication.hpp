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


#ifndef CRAP_LINKQUAD_SERIAL_COMMUNICATION
#define CRAP_LINKQUAD_SERIAL_COMMUNICATION

#include <pthread.h>    // POSIX threads
#include <stdint.h> // type definitions
#include <cstdio>
#include <map>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "serial_base.hpp"
#include "serial_data.hpp"
#include "serial_listener.hpp"
#include "serial_talker.hpp"
#include "PP/repeat.hpp"
#include "PP/comma_if.hpp"
#include "PP/plus_if.hpp"


namespace LinkQuad {
    namespace comm {
        namespace serial {
// LinkAPI
//~ #include "link_api/packet.h"         // parsing and packet format
//~ #include "link_api/packet_types_defines.h"
//~ #include "link_api/messages.h"       // definitions of message types

            typedef std::map<std::string, int> fdmap;
            extern fdmap serial_ports;
            extern pthread_mutex_t portmap_lock;

            typedef std::map<std::string, serial_base*> listener_map;
            extern listener_map serial_listeners;

            #define BAUDRATE B115200

            inline int configure_port(const std::string portname)
            {
                int fd = serial_ports[portname];
                if(fd > 0) return fd;
                fd = open(portname.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK /*| O_NDELAY*/);
                if(fd < 0) {
                    std::cerr << "ERROR: Device NOT open!\nCheck the port name (" << portname << ") and try again" << std::endl;
                    exit(0);
                }
                //Set all the options
                struct termios termios_opt;

                bzero(&termios_opt, sizeof(termios_opt));

                termios_opt.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
                termios_opt.c_iflag = IGNPAR | IXOFF;
                termios_opt.c_oflag = 0;

                tcsetattr(fd,TCSANOW,&termios_opt);

                tcflush(fd, TCIOFLUSH);

                return fd;
            }


            #define tpldef(z,n,extra) PP_COMMA_IF(n) typename T##n extra
            #define tplname(z,n,q) PP_COMMA_IF(n) T##n
            template<PP_REPEAT(MAX_NUM_DATAMEMBERS, tpldef, = data::nothing)>
            void send(const std::string portname, const data::serial_data<PP_REPEAT(MAX_NUM_DATAMEMBERS, tplname,)>& message) {
                typedef data::serial_data<PP_REPEAT(MAX_NUM_DATAMEMBERS, tplname,)> datatype;

                pthread_mutex_lock(&portmap_lock);
                    int fd = serial_ports[portname];
                    if(!fd) {
                        fd = serial_ports[portname] = configure_port(portname);
                    }
                pthread_mutex_unlock(&portmap_lock);

                new serial_talker<datatype>(message, fd);
            }


            template<typename CMD_TYPE, PP_REPEAT(MAX_NUM_DATAMEMBERS, tpldef, = data::nothing)>
            void listen(const std::string portname, void(*callback)(const data::serial_data<PP_REPEAT(MAX_NUM_DATAMEMBERS, tplname,)>), const unsigned int every_nth = 1) {
                typedef data::serial_data<PP_REPEAT(MAX_NUM_DATAMEMBERS, tplname,)> datatype;
                delete serial_listeners[portname];

                pthread_mutex_lock(&portmap_lock);
                    int fd = serial_ports[portname];
                    if(!fd) {
                        fd = serial_ports[portname] = configure_port(portname);
                    }
                pthread_mutex_unlock(&portmap_lock);

                CMD_TYPE request(every_nth, datatype::get_ids(), datatype::get_count());
                using namespace data;
                send(portname, static_cast<const serial_data<request_part::everyNth_t, request_part::ids_t<sizeof(CMD_TYPE::ids)>, request_part::ids_cnt_t>& >(request));
                serial_listeners[portname] = new serial_listener<datatype>(callback, fd);
            }

            #undef tpln
        }
    }
}

#endif
