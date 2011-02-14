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
 * \file src/elliptic-directpath.cc
 *
 * \brief Implementation of EllipticDirectPath.
 */

#include <math.h>

#include "kws/hash-optimizer/distance.hh"

#include "kws/hash-optimizer/elliptic-directpath.hh"

namespace kws
{
  namespace hashoptimizer
  {
    EllipticDirectPath::
    ~EllipticDirectPath ()
    {
    }

    ktStatus
    EllipticDirectPath::init (const EllipticDirectPathWkPtr& inWeakPtr)
    {
      attWeakPtr_ = inWeakPtr;
      return KD_OK;
    }

    EllipticDirectPathShPtr EllipticDirectPath::
    create (const CkwsConfig& i_start,
	    const CkwsConfig& i_end,
	    const DistanceShPtr& i_distance,
	    const CkwsSteeringMethodShPtr& i_steeringMethod)
    {
      EllipticDirectPath* ptr
	= new EllipticDirectPath (i_start, i_end, i_distance, i_steeringMethod);
      EllipticDirectPathShPtr shPtr(ptr);

      if (KD_OK != ptr->init (shPtr))
	shPtr.reset ();
      
      return shPtr;
    }

    EllipticDirectPathShPtr EllipticDirectPath::
    createCopy (const EllipticDirectPathConstShPtr& i_directPath)
    {
      if (i_directPath)
	{
	  EllipticDirectPath* pathPtr = new EllipticDirectPath (*i_directPath);
	  EllipticDirectPathShPtr pathShPtr (pathPtr);
	  EllipticDirectPathWkPtr pathWkPtr (pathShPtr);
	  
	  if (pathPtr->init (pathWkPtr) != KD_OK)
	    {
	      pathShPtr.reset() ;
	    }
	  return pathShPtr;
	}
      else return EllipticDirectPathShPtr() ;
    }

    CkwsAbstractPathShPtr EllipticDirectPath::
    clone () const
    {
      return EllipticDirectPath::createCopy(attWeakPtr_.lock());
    }
      
    double EllipticDirectPath::
    computePrivateLength () const
    {
      return attDistance->distance (startConfiguration (), endConfiguration ());
    }
      
    void EllipticDirectPath::
    interpolate (double i_s, CkwsConfig& o_cfg) const
    {
      CkwsConfig endCfg = endConfiguration ();
      if (i_s >= length ())
	{
	  o_cfg.dofValue (0, endCfg.dofValue (0));
	  o_cfg.dofValue (1, endCfg.dofValue (1));
	  o_cfg.dofValue (2, endCfg.dofValue (2));
	  o_cfg.dofValue (3, endCfg.dofValue (3));
	  o_cfg.dofValue (4, endCfg.dofValue (4));
	  o_cfg.dofValue (5, endCfg.dofValue (5));
	  return;
	}

      CkwsConfig startCfg = startConfiguration ();

      double deltaX = endCfg.dofValue (0) - startCfg.dofValue (0);
      double deltaY = endCfg.dofValue (1) - startCfg.dofValue (1);
      double vecNorm = sqrt (pow (deltaX, 2) + pow (deltaY, 2));

      const unsigned int stepsNb = (int)(vecNorm
					 / attDistance->integrationStep ());
      unsigned int stepIndex = 0;
      double timeDistance = 0.;

      do
	{
	  o_cfg.dofValue (0, startCfg.dofValue (0) * (1 - timeDistance / length ())
			  + endCfg.dofValue (0) * timeDistance / length ());
	  o_cfg.dofValue (1, startCfg.dofValue (1) * (1 - timeDistance / length ())
			  + endCfg.dofValue (1) * timeDistance / length ());
	  o_cfg.dofValue (2, startCfg.dofValue (2));
	  o_cfg.dofValue (3, startCfg.dofValue (3));
	  o_cfg.dofValue (4, startCfg.dofValue (4));
	  o_cfg.dofValue (5, startCfg.dofValue (5) * (1 - timeDistance / length ())
			  + endCfg.dofValue (5) * timeDistance / length ());

	  const double dt = attDistance->elementaryCost (startCfg, endCfg,
							 stepsNb, stepIndex);
	  timeDistance += dt;
	  stepIndex++;
	}
      while (timeDistance < i_s && stepIndex < stepsNb);

      return;
    }
      
    void EllipticDirectPath:: 
    maxAbsoluteDerivative(double i_from,
			  double i_to,
			  std::vector<double>& o_derivative) const
    {
      o_derivative.push_back (1.0);
      o_derivative.push_back (1.0);
      o_derivative.push_back (1.0);
      o_derivative.push_back (1.0);
      o_derivative.push_back (1.0);
      o_derivative.push_back (1.0);
    }

    EllipticDirectPath::
    EllipticDirectPath (const CkwsConfig& i_start,
			const CkwsConfig& i_end,
			const DistanceShPtr& i_distance,
			const CkwsSteeringMethodShPtr& i_steeringMethod)
      : CkwsDirectPath (i_start, i_end, i_steeringMethod)
    {
      attDistance = i_distance;
    }    

  } // end of namespace hashoptimizer.
} // end of namespace kws.
