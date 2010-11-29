// Copyright (C) 2010 by Antonio El Khoury.
//
// This file is part of the kws-hash-optimizer.
//
// kws-hash-optimizer is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either
// version 3 of the License, or (at your option) any later version.
//
// kws-hash-optimizer is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with kws-hash-optimizer.  If not, see
// <http://www.gnu.org/licenses/>.


/**
 * \file src/directpath.cc
 *
 * \brief Implementation of DirectPath.
 */

#include <math.h>

#include "kws/hash-optimizer/directpath.hh"

namespace kws
{
  namespace hashoptimizer
  {
    // DirectPathShPtr DirectPath::
    // clone () const
    // {
    // }
      
    double DirectPath::
    computePrivateLength () const
    {
      return 0;
    } 
      
    void DirectPath::
    interpolate (double i_s, CkwsConfig& o_cfg) const
    {
    }
      
    void DirectPath:: 
    maxAbsoluteDerivative(double i_from,
			  double i_to,
			  std::vector<double>& o_derivative) const
    {
    }
      } // end of namespace hashoptimizer.
} // end of namespace kws.
