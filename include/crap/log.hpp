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

#ifndef CRAP_LOG_HPP_
#define CRAP_LOG_HPP_

#include <boost/current_function.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>

#define LOG_INFO ::CRAP::log::logger(::CRAP::log::s_info, __FILE__, BOOST_CURRENT_FUNCTION, __LINE__)
#define LOG_WARNING ::CRAP::log::logger(::CRAP::log::s_warning, __FILE__, BOOST_CURRENT_FUNCTION, __LINE__)
#define LOG_ERROR ::CRAP::log::logger(::CRAP::log::s_error, __FILE__, BOOST_CURRENT_FUNCTION, __LINE__)

namespace CRAP {
    namespace log {
        enum severity_t {
            s_info,
            s_warning,
            s_error
        };
        const char* severity_t_map[] = {"INFO", "WARNING", "ERROR"};
        enum color_t {
            no_color = -1,
            black = 0,
            red = 1,
            green = 2,
            yellow = 3,
            blue = 4,
            magenta = 5,
            cyan = 6,
            white = 7
        };
        const CRAP::log::color_t severity_colormap[] = {
            green, // info
            yellow, // warning
            red // error
        };

        inline std::string bash_color(const color_t fg, const bool bold = false, const color_t bg = no_color) {
            std::string s;
            s += (bold ? "1;" : "0;");
            if(fg != no_color) { s += "3"; s += boost::lexical_cast<std::string>(fg); s += ";"; }
            if(bg != no_color) { s += "4"; s += boost::lexical_cast<std::string>(bg); s += ";"; }
            s.erase(s.size()-1);
            return "\E[" + s + "m";
        }
        inline std::string bash_color() {
            return "\E[0m";
        }


        class logger {
            private:
                severity_t severity;
                std::string file;
                std::string function;
                int line;

            public:
                logger(severity_t s, const std::string file_, const std::string function_, const int line_) {
                    severity = s;
                    file = file_;
                    function = function_;
                    line = line_;
                }

                void operator<<(const std::string msg) {
                    std::cout
                    << bash_color(magenta) << "Log ["
                    << bash_color(severity_colormap[severity])
                    << severity_t_map[severity]
                    << bash_color(magenta) << "]: "
                    << bash_color() << file
                    << bash_color() << " (" << function << ":" << line << ")"
                    << bash_color(cyan) << " | "
                    << bash_color()
                    << bash_color(no_color, true) << msg
                    << bash_color() << std::endl;
                }
        };

    }
}

#endif