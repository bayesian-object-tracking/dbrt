/*************************************************************************
This software allows for filtering in high-dimensional observation and
state spaces, as described in

M. Wuthrich, P. Pastor, M. Kalakrishnan, J. Bohg, and S. Schaal.
Probabilistic Object Tracking using a Range Camera
IEEE/RSJ Intl Conf on Intelligent Robots and Systems, 2013

In a publication based on this software pleace cite the above reference.


Copyright (C) 2014  Manuel Wuthrich

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*************************************************************************/

#ifndef FAST_FILTERING_UTILS_ASSERTIONS_HPP
#define FAST_FILTERING_UTILS_ASSERTIONS_HPP

#include <boost/mpl/assert.hpp>
#include <boost/type_traits/is_base_of.hpp>


/**
 * \ingroup macros
 * \internal
 *
 * This variadic macro performs a compile time derivation assertion. The first
 * argument is the derived type which is being tested whether it implements a
 * base type. The base type is given as the second argument list.
 *
 * __VA_ARGS__ was used as a second parameter to enable passing template
 * specialization to the macro.
 *
 * Note: The macro requires <em>derived_type</em> to be a single worded type. In
 *       case of a template specialization, use a typedef.
 */
#define REQUIRE_INTERFACE(derived_type, ...)\
    BOOST_STATIC_ASSERT_MSG(( \
            boost::is_base_of<__VA_ARGS__, derived_type>::value), \
            #derived_type " must implement " #__VA_ARGS__ " interface.");

#endif
