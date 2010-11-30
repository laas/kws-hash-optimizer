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
    DirectPath::
    ~DirectPath ()
    {
    }

    ktStatus
    DirectPath::init (const DirectPathWkPtr& inWeakPtr)
    {
      attWeakPtr_ = inWeakPtr;
      return KD_OK;
    }

    DirectPathShPtr DirectPath::
    create (const CkwsConfig& i_start,
	    const CkwsConfig& i_end,
	    const CkwsSteeringMethodShPtr& i_steeringMethod)
    {
      DirectPath* ptr = new DirectPath (i_start, i_end, i_steeringMethod);
      DirectPathShPtr shPtr(ptr);

      if (KD_OK != ptr->init (shPtr))
	shPtr.reset ();
      
      return shPtr;
    }

    DirectPathShPtr DirectPath::
    createCopy (const DirectPathConstShPtr& i_directPath)
    {
      if (i_directPath)
	{
	  DirectPath* pathPtr = new DirectPath (*i_directPath);
	  DirectPathShPtr pathShPtr (pathPtr);
	  DirectPathWkPtr pathWkPtr (pathShPtr);
	  
	  if (pathPtr->init (pathWkPtr) != KD_OK)
	    {
	      pathShPtr.reset() ;
	    }
	  return pathShPtr;
	}
      else return DirectPathShPtr() ;
    }

    CkwsAbstractPathShPtr DirectPath::
    clone () const
    {
      return DirectPath::createCopy(attWeakPtr_.lock());
    }
      
    double DirectPath::
    computePrivateLength () const
    {
      CkwsConfig startCfg (device ());
      getConfigAtStart (startCfg);
      CkwsConfig endCfg (device ());
      getConfigAtEnd (endCfg);

      double deltaX = endCfg.dofValue (0) - startCfg.dofValue (0);
      double deltaY = endCfg.dofValue (1) - startCfg.dofValue (1);
      double vecNorm = sqrt (pow (deltaX, 2) + pow (deltaY, 2));

      return vecNorm;
    }
      
    void DirectPath::
    interpolate (double i_s, CkwsConfig& o_cfg) const
    {
      CkwsConfig startCfg (device ());
      getConfigAtStart (startCfg);
      CkwsConfig endCfg (device ());
      getConfigAtStart (endCfg);

      double deltaX = endCfg.dofValue (0) - startCfg.dofValue (0);
      double deltaY = endCfg.dofValue (1) - startCfg.dofValue (1);

      o_cfg.dofValue (0, i_s * deltaY / deltaX);
      o_cfg.dofValue (1, i_s * deltaX / deltaY);
      o_cfg.dofValue (2, startCfg.dofValue (2));
      o_cfg.dofValue (3, startCfg.dofValue (3));
      o_cfg.dofValue (4, startCfg.dofValue (4));
      o_cfg.dofValue (5, startCfg.dofValue (5));
    }
      
    void DirectPath:: 
    maxAbsoluteDerivative(double i_from,
			  double i_to,
			  std::vector<double>& o_derivative) const
    {
      o_derivative[0] = 1;
      o_derivative[1] = 1;
      o_derivative[2] = 0;
      o_derivative[3] = 0;
      o_derivative[4] = 0;
      o_derivative[5] = 0;
    }

    DirectPath::
    DirectPath (const CkwsConfig& i_start,
		const CkwsConfig& i_end,
		const CkwsSteeringMethodShPtr& i_steeringMethod)
      : CkwsDirectPath (i_start, i_end, i_steeringMethod)
    {
    }    

  } // end of namespace hashoptimizer.
} // end of namespace kws.
