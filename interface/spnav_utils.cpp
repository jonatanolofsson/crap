#include <iostream>
#include <cstdlib>
#include <signal.h>
#include <boost/thread.hpp>
#include "spnav_utils.hpp"

namespace spnav {
    boost::thread reader_thread;

    void sig(int)
    {
        spnav_close();
        exit(0);
    }

    void readloop(void(*callback)(spnav_event&)) {
        spnav_event sev;

        signal(SIGINT, sig);

        if(spnav_open()==-1) {
            std::cerr << "Failed to connect to the space navigator daemon\n" << std::endl;
        }

        /* spnav_wait_event() and spnav_poll_event(), will silently ignore any non-spnav X11 events.
         *
         * If you need to handle other X11 events you will have to use a regular XNextEvent() loop,
         * and pass any ClientMessage events to spnav_x11_event, which will return the event type or
         * zero if it's not an spnav event (see spnav.h).
         */
        while(spnav_wait_event(&sev)) {
            callback(sev);
        }

        spnav_close();
        return;
    }

    void listen(void(*callback)(spnav_event&))
    {
        boost::thread(boost::bind(readloop, callback));
    }
}
