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


#ifndef CRAP_LINKQUAD_SERIAL_DATA
#define CRAP_LINKQUAD_SERIAL_DATA

#include <cstring>
#include <string>
#include <assert.h>

#include "PP/repeat.hpp"
#include "PP/comma_if.hpp"
#include "PP/plus_if.hpp"

#include <stdint.h> // type definitions
#include <iostream>


#define MAX_NUM_DATAMEMBERS 158

namespace LinkQuad {
    namespace comm {
        namespace serial {
#include "link_api/messages.h"
#include "link_api/packet.h"
            namespace data {
                struct nothing { static const uint8_t c = 0; static const uint8_t size = 0; static const uint8_t n = 0; static void set(const uint8_t*){} static void get(uint8_t*){}};

                namespace request_part {
                    template<int TYPE, int N> struct DataRequestPart;
                }

                #include "converted.hpp"

                
                #define tpldef(z,n,extra) PP_COMMA_IF(n) typename T##n extra
                 #define _nothing(z,n,c) PP_COMMA_IF(n) nothing
                  #define getn(z,w,d) PP_COMMA_IF(w) T##w::n
                   #define declv(z,w,d) PP_COMMA_IF(w) T##w v##w = NULL
                    #define setv(z,w,d) T##w::set(p);
                     #define getv(z,w,d) T##w::get(p);
                      #define szof(z,w,d) PP_PLUS_IF(w) T##w::size
                       #define getc(z,w,d) PP_PLUS_IF(w) T##w::c
                        #define tplname(z,n,q) PP_COMMA_IF(n) T##n
                        template<typename firstT = nothing, PP_REPEAT(MAX_NUM_DATAMEMBERS, tpldef, = nothing)> struct serial_data;
                        template<> struct serial_data<nothing, PP_REPEAT(MAX_NUM_DATAMEMBERS, _nothing,)> {};
                        template<typename firstT, PP_REPEAT(MAX_NUM_DATAMEMBERS, tpldef,)>
                        struct serial_data : public firstT, public serial_data<PP_REPEAT(MAX_NUM_DATAMEMBERS, tplname,)> {
                            static const uint8_t* get_ids() {
                                static const uint8_t _ids[MAX_NUM_DATAMEMBERS+1] = {firstT::n, PP_REPEAT(MAX_NUM_DATAMEMBERS, getn,)};
                                return _ids;
                            }
                            static int get_count() {
                                return firstT::c + PP_REPEAT(MAX_NUM_DATAMEMBERS, getc,);
                            }
                            packet_metadata_t metadata;
                            bool set(const uint8_t* p, size_t size) {
                                if(size != ( firstT::size + PP_REPEAT(MAX_NUM_DATAMEMBERS, szof, 0))) {
                                    std::cout << ( firstT::size + PP_REPEAT(MAX_NUM_DATAMEMBERS, szof, 0)) << " vs " << size << std::endl;
                                    return false;
                                }
                                firstT::set(p); PP_REPEAT(MAX_NUM_DATAMEMBERS, setv,0)
                                return true;
                            }
                            size_t marshal(unsigned char* p) {
                                unsigned char* q = p;
                                this->firstT::get(p); PP_REPEAT(MAX_NUM_DATAMEMBERS, getv,0)
                                return (size_t) (p-q);
                            }

                            serial_data() {
                               metadata.source = PACKET_UNDEFINED;
                               metadata.destination = PACKET_UNDEFINED;
                               metadata.sequence_number = PACKET_UNDEFINED;
                               metadata.packet_type = PACKET_UNDEFINED;
                            }
                        };
                       #undef getc
                      #undef szof
                     #undef getv
                    #undef setv
                   #undef declv
                  #undef getn
                 #undef _nothing
                #undef tpln



                namespace request_part {
                    template<int TYPE, int N>
                    struct DataRequestPart : public serial_data<request_part::everyNth_t, request_part::ids_t<N>, request_part::ids_cnt_t>
                    {
                        static const int NUM_IDS = N;
                        DataRequestPart(const uint8_t everyNth_, const uint8_t* ids_, const uint8_t ids_cnt_) 
                            : serial_data<request_part::everyNth_t, request_part::ids_t<N>, request_part::ids_cnt_t>::serial_data() 
                        {
                            assert(ids_cnt_ <= N);
                            this->everyNth = everyNth_;
                            memset(this->ids, 0, sizeof(this->ids));
                            memcpy(this->ids, ids_, ids_cnt_);
                            this->ids_cnt = ids_cnt_;
                            this->metadata.packet_type = TYPE;
                        }
                    };
                }
            }
        }
    }
}

#endif
