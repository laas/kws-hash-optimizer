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
#include <sstream>

#include <hpp/util/debug.hh>

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
	    const DistanceConstShPtr& i_distance,
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
      double duration = boost::get<0> (attPiecewiseSpeedVector_.back ());
      
      return duration;
    }
    
    double EllipticDirectPath::circularDistance(double i_a1, double i_a2) const 
    {
      const double angle = i_a2 - i_a1;
  
      double result = fmod (angle, 2.*M_PI);
  
      // 'result' is now inside ]-2.pi, 2.pi[
  
      if (result >= M_PI)
	{
	  result -= 2.*M_PI;
	}
      else if (result < -M_PI)
	{
	  result += 2.*M_PI;
	}
  
      return result;
    }
  
    void EllipticDirectPath::
    interpolate (double i_s, CkwsConfig& o_cfg) const
    {
      CkwsConfig endCfg = endConfiguration ();
      // if (i_s >= length ())
      // 	{
      // 	  o_cfg.dofValue (0, endCfg.dofValue (0));
      // 	  o_cfg.dofValue (1, endCfg.dofValue (1));
      // 	  o_cfg.dofValue (2, endCfg.dofValue (2));
      // 	  o_cfg.dofValue (3, endCfg.dofValue (3));
      // 	  o_cfg.dofValue (4, endCfg.dofValue (4));
      // 	  o_cfg.dofValue (5, endCfg.dofValue (5));
	  
      // 	  return;
      // 	}
           
      // Run a brinary search to find the correct time interval.
      unsigned int min = 0;
      unsigned int max = attPiecewiseSpeedVector_.size () - 1;
      unsigned int mid = 0;
      unsigned int timeIndex;

      while (min < max - 1)
                       	{
	  mid = min + (max - min) / 2;
	  if (i_s > boost::get<0> (attPiecewiseSpeedVector_.at (mid)))
	    {
	      min = mid;
	    }
	  else
	    {
	      max = mid;
	    }
	}
      timeIndex = min;

      // Retrieve information at time index.
      const double time
      	= boost::get<0> (attPiecewiseSpeedVector_.at (timeIndex));
      const CkwsConfig sampleCfg
      	= boost::get<1> (attPiecewiseSpeedVector_.at (timeIndex));
      const SpeedVector boxFrameSpeed
      	= boost::get<2> (attPiecewiseSpeedVector_.at (timeIndex));

      //Express speed in world frame.
      const double boxAngle = sampleCfg.dofValue (5);
      const double vf = boxFrameSpeed.at (0);
      const double vlat = boxFrameSpeed.at (1);
      
      const double vx = vf * cos (boxAngle)
      	- vlat * sin (boxAngle);
      const double vy =  vlat * cos (boxAngle)
      	+ vf * sin (boxAngle);

      // Compute interpolated configuration.
      const double interpolateInterval = i_s - time;
      
      o_cfg.dofValue (0, sampleCfg.dofValue (0) + vx * interpolateInterval);
      o_cfg.dofValue (1, sampleCfg.dofValue (1) + vy * interpolateInterval);
      o_cfg.dofValue (2, sampleCfg.dofValue (2));
      o_cfg.dofValue (3, sampleCfg.dofValue (3));
      o_cfg.dofValue (4, sampleCfg.dofValue (4));

      // Compute angle by linear interpolation.
      const double nextTime
      	= boost::get<0> (attPiecewiseSpeedVector_.at (timeIndex + 1));
      const CkwsConfig nextSampleCfg
      	= boost::get<1> (attPiecewiseSpeedVector_.at (timeIndex + 1));
      const double nextBoxAngle = nextSampleCfg.dofValue (5);

      const double sampleInterval = nextTime - time;
      const double timeRatio = interpolateInterval / sampleInterval;

      o_cfg.dofValue (5, boxAngle
		      + timeRatio * circularDistance (boxAngle, nextBoxAngle));

      // hppDout (notice, time << "\t" << sampleCfg.dofValue (0) 
      // 	       << "\t" << o_cfg.dofValue (0) << "\t" << time << "\t" << i_s << "\t" << vx << "\t" << vf << "\t" << vlat << "\t" << boxAngle << "\t" << nextBoxAngle);

      // hppDout (notice, (boost::get<1> (attPiecewiseSpeedVector_.at (0))).dofValue (5) << "\t"
      // 	       << (boost::get<1> (attPiecewiseSpeedVector_.at (1))).dofValue (5) << "\t"
      // 	       << (boost::get<1> (attPiecewiseSpeedVector_.at (2))).dofValue (5) << "\t");
      
      // CkwsConfig startCfg = startConfiguration ();

      // double deltaX = endCfg.dofValue (0) - startCfg.dofValue (0);
      // double deltaY = endCfg.dofValue (1) - startCfg.dofValue (1);
      // double vecNorm = sqrt (pow (deltaX, 2) + pow (deltaY, 2));

      // // DistanceConstShPtr dist
      // // 	= KIT_DYNAMIC_PTR_CAST (Distance const, attDistance);
      
      // // if (!dist)
      // // 	hppDout(notice, "failllll2");

      // const unsigned int stepsNb = (int)(vecNorm
      // 					 / attDistance_->integrationStep ());
      // unsigned int stepIndex = 0;
      // double timeDistance = 0.;

      // do
      // 	{
      // 	  o_cfg.dofValue (0, startCfg.dofValue (0) * (1 - timeDistance / length ())
      // 			  + endCfg.dofValue (0) * timeDistance / length ());
      // 	  o_cfg.dofValue (1, startCfg.dofValue (1) * (1 - timeDistance / length ())
      // 			  + endCfg.dofValue (1) * timeDistance / length ());
      // 	  o_cfg.dofValue (2, startCfg.dofValue (2));
      // 	  o_cfg.dofValue (3, startCfg.dofValue (3));
      // 	  o_cfg.dofValue (4, startCfg.dofValue (4));
      // 	  o_cfg.dofValue (5, startCfg.dofValue (5) * (1 - timeDistance / length ())
      // 			  + endCfg.dofValue (5) * timeDistance / length ());

      // 	  const double dt = attDistance_->elementaryCost (startCfg, endCfg,
      // 							 stepsNb, stepIndex);
      // 	  timeDistance += dt;
      // 	  stepIndex++;
      // 	}
      // while (timeDistance < i_s && stepIndex < stepsNb);

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

    void EllipticDirectPath::
    buildPiecewiseSpeedVector (const CkwsConfig& i_start,
			       const CkwsConfig& i_end,
			       PiecewiseSpeedVector& o_piecewiseSpeed) const
    {
      double deltaX = i_end.dofValue (0) - i_start.dofValue (0);
      double deltaY = i_end.dofValue (1) - i_start.dofValue (1);
      double vecNorm = sqrt (pow (deltaX, 2) + pow (deltaY, 2));

      const unsigned int stepsNb = ceil (vecNorm
      					 / attDistance_->integrationStep ());
      unsigned int stepIndex = 0;
      double timeDistance = 0.;
      
      while (stepIndex < stepsNb)
	{
	  CkwsConfig sampleCfg (device ());
	  SpeedVector speedVector;

	  const double dt = attDistance_->elementaryCost (i_start, i_end,
							  stepsNb, stepIndex,
							  sampleCfg,
							  speedVector);
	  
	  const TimeSample timeSample (timeDistance, sampleCfg, speedVector);
	  //hppDout (notice, timeDistance << "\t" << sampleCfg.dofValue (0));
	  o_piecewiseSpeed.push_back (timeSample);
	  
      	  timeDistance += dt;
      	  stepIndex++;
	}
      SpeedVector speedVector;
      speedVector.push_back (0.0);
      speedVector.push_back (0.0);
      const TimeSample timeSample (timeDistance, endConfiguration (), speedVector);
      //hppDout (notice, timeDistance << "\t" << endConfiguration ().dofValue (0));
      o_piecewiseSpeed.push_back (timeSample);
      
      return;
    }

    EllipticDirectPath::
    EllipticDirectPath (const CkwsConfig& i_start,
			const CkwsConfig& i_end,
			const DistanceConstShPtr& i_distance,
			const CkwsSteeringMethodShPtr& i_steeringMethod)
      : CkwsDirectPath (i_start, i_end, i_steeringMethod)
    {
      attDistance_ = i_distance;
      buildPiecewiseSpeedVector (i_start, i_end, attPiecewiseSpeedVector_);
    }    
  } // end of namespace hashoptimizer.
} // end of namespace kws.
