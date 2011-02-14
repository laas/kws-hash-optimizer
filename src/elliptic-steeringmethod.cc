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
 * \file src/elliptic-steeringmethod.cc
 *
 * \brief Implementation of EllipticSteeringMethod.
 */

#include "kws/hash-optimizer/elliptic-directpath.hh"

#include "kws/hash-optimizer/elliptic-steeringmethod.hh"

namespace kws
{
  namespace hashoptimizer
  {
    CkwsDirectPathShPtr EllipticSteeringMethod::
    makeDirectPath (const CkwsConfig& i_startCfg,
		    const CkwsConfig& i_endCfg)
    {
      return EllipticDirectPath::create (i_startCfg, i_endCfg, attDistance,
					 attWeakPtr_.lock ());
    }

    bool EllipticSteeringMethod::
    isOriented () const
    {
      return false;
    }

    ktStatus EllipticSteeringMethod::
    init (const EllipticSteeringMethodWkPtr &i_smWkPtr)
    {
      if (CkwsSteeringMethod::init (i_smWkPtr) != KD_OK)
	return KD_ERROR;
      attWeakPtr_ = i_smWkPtr;

      return KD_OK;
    }
 
    EllipticSteeringMethodShPtr EllipticSteeringMethod::
    create (const DistanceShPtr& i_distance, bool i_oriented)
    {
      EllipticSteeringMethod* ptr = new EllipticSteeringMethod(i_distance, i_oriented);          
      EllipticSteeringMethodShPtr shPtr(ptr);
      
      if(ptr->init(shPtr) != KD_OK)
	shPtr.reset();
      
      return shPtr;                                                                 
    }

    EllipticSteeringMethod::
    EllipticSteeringMethod (const DistanceShPtr& i_distance, bool i_oriented)
  : CkwsSteeringMethod::CkwsSteeringMethod ()
    {
      attDistance = i_distance;
    }

  } // end of namespace hashoptimizer.
} // end of namespace kws.
