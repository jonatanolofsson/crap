
#ifndef SPNAV_UTILS_HPP_
#define SPNAV_UTILS_HPP_
#include <spnav.h>

namespace spnav {
    void readloop(void(*callback)(spnav_event&));
    void listen(void(*callback)(spnav_event&));
    typedef spnav_event event;
}

#endif
