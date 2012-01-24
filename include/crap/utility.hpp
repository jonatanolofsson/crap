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

#ifndef CRAP_UTILITY_HPP_
#define CRAP_UTILITY_HPP_

#include <sstream>

namespace CRAP {
    namespace utility {

        /*!
         * \brief   Calculate the linear array index of a two dimensional matrix
         * \param   x   Matrix row position
         * \param   y   Matrix column position
         * \return  Linear array index
         */
        template<unsigned STA, unsigned LDA>
        inline unsigned index(int x,int y) {
            return x + STA + LDA*y;
        }


        template <class T>
        inline std::string to_string (const T& t)
        {
            std::stringstream ss;
            ss << t;
            return ss.str();
        }
    }
}

#endif
