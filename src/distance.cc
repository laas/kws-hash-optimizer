// Copyright (C) 2008, 2009 by Antonio El Khoury, CNRS.
//
// This file is part of the kws-hash-optimizer.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#include <fstream>
#include <math.h>

#include <KineoWorks2/kwsPath.h>
#include <KineoWorks2/kwsDevice.h>
#include <KineoWorks2/kwsDof.h>
#include <KineoWorks2/kwsSMLinear.h>
#include <KineoWorks2/kwsDPLinear.h>

#include <hpp/util/debug.hh>
#include <hpp/util/indent.hh>
#include <hpp/util/timer.hh>

#include "kws/hash-optimizer/directpath.hh"
#include "kws/hash-optimizer/steeringmethod.hh"

#include "kws/hash-optimizer/distance.hh"

// Namespaces.

namespace kws
{
  namespace hashoptimizer
  {
    Distance::~Distance ()
    {
    }

    DistanceShPtr Distance::create (const double& i_integrationStep,
				    const double& i_vMinX,
				    const double& i_vMaxX,
				    const double& i_vMaxY)
    {
      Distance* distance = new Distance (i_integrationStep,
					 i_vMinX, i_vMaxX, i_vMaxY);
      DistanceShPtr distanceShPtr (distance);

      if (distance->init (distanceShPtr) != KD_OK) {
	distanceShPtr.reset ();
      }
      return distanceShPtr;
    }

    Distance::Distance (const double& i_integrationStep,
			const double& i_vMinX,
			const double& i_vMaxX,
			const double& i_vMaxY)
      : CkwsDistance ()
    {
      attIntegrationStep = i_integrationStep;
      attVMinX = i_vMinX;
      attVMaxX = i_vMaxX;
      attVMaxY = i_vMaxY;
    }

    ktStatus Distance::init(DistanceWkPtr inWeakPtr)
    {
      if (CkwsDistance::init(inWeakPtr) != KD_OK) {
	return KD_ERROR;
      }
      attWeakPtr = inWeakPtr;
      return KD_OK;
    }

    void Distance::
    sampleConfiguration (const CkwsDirectPathShPtr& i_directPath,
			 const unsigned int i_stepsNb,
			 const unsigned int i_stepIndex,
			 CkwsConfig& o_cfg) const
    {
      if (i_stepIndex < i_stepsNb)
	{
	  i_directPath->getConfigAtDistance (i_stepIndex * attIntegrationStep,
					     o_cfg);
	}
      else
	{	    
	  i_directPath->getConfigAtEnd (o_cfg);
	}

      return;
    }

    double Distance::
    elementaryCost (const CkwsConfig &i_cfg1,
		    const CkwsConfig &i_cfg2,
		    const unsigned int i_stepsNb,
		    const unsigned int i_stepIndex) const
    {
      CkwsSteeringMethodShPtr steeringMethod = SteeringMethod::create ();
      CkwsDirectPathShPtr directPath
	= steeringMethod->makeDirectPath (i_cfg1, i_cfg2);
      CkwsDeviceShPtr boxDevice = directPath->device ();

      CkwsConfig sampleStartCfg (boxDevice);
      directPath->getConfigAtDistance (i_stepIndex * attIntegrationStep,
				       sampleStartCfg);
      CkwsConfig sampleEndCfg (boxDevice);
      if (i_stepIndex < i_stepsNb)
      	directPath->getConfigAtDistance ((i_stepIndex + 1) * attIntegrationStep,
      					   sampleEndCfg);
      else
      	directPath->getConfigAtEnd (sampleEndCfg);

      CkwsDirectPathShPtr elementaryDP
	= steeringMethod->makeDirectPath (sampleStartCfg, sampleEndCfg);
      const double integrationStep = elementaryDP->length ();

      // Compute frontal and lateral speeds.
      const double dx = sampleEndCfg.dofValue (0)
	- sampleStartCfg.dofValue (0);
      const double dy = sampleEndCfg.dofValue (1)
	- sampleStartCfg.dofValue (1);

      const double theta = sampleStartCfg.dofValue (5);

      double vf = dx * std::cos (theta)
	+ dy * std::sin (theta);
      double vlat =  dy * std::cos (theta)
	- dx * std::sin (theta);

      // Compute speed vector intersection with the speed cosntraint

      const double tangent = vlat / vf;
	  
      vf = vf >= 0 ?
	1 / sqrt (1 / (attVMaxX * attVMaxX)
		  + (tangent * tangent) / (attVMaxY * attVMaxY))
	: - 1 / sqrt (1 / (attVMinX * attVMinX)
		    + (tangent * tangent) / (attVMaxY * attVMaxY));
      vlat = tangent * vf;
      
      // Compute elementary walk time.
      const double dt = integrationStep / sqrt (vf * vf + vlat * vlat);

      return dt; 
    }

    double Distance::
    elementaryCost (const CkwsConfig &i_cfg1,
		    const CkwsConfig &i_cfg2,
		    const unsigned int i_stepsNb,
		    const unsigned int i_stepIndex,
		    CkwsConfig& o_cfg,
		    SpeedVector& o_speed) const
    {
      CkwsSteeringMethodShPtr steeringMethod = SteeringMethod::create ();
      CkwsDirectPathShPtr directPath
	= steeringMethod->makeDirectPath (i_cfg1, i_cfg2);
      CkwsDeviceShPtr boxDevice = directPath->device ();

      CkwsConfig sampleStartCfg (boxDevice);
      directPath->getConfigAtDistance (i_stepIndex * attIntegrationStep,
				       sampleStartCfg);
      o_cfg = sampleStartCfg;

      CkwsConfig sampleEndCfg (boxDevice);
      if (i_stepIndex < i_stepsNb)
      	directPath->getConfigAtDistance ((i_stepIndex + 1) * attIntegrationStep,
      					   sampleEndCfg);
      else
      	directPath->getConfigAtEnd (sampleEndCfg);

      CkwsDirectPathShPtr elementaryDP
	= steeringMethod->makeDirectPath (sampleStartCfg, sampleEndCfg);
      const double integrationStep = elementaryDP->length ();

      // Compute frontal and lateral speeds.
      const double dx = sampleEndCfg.dofValue (0)
	- sampleStartCfg.dofValue (0);
      const double dy = sampleEndCfg.dofValue (1)
	- sampleStartCfg.dofValue (1);

      const double theta = sampleStartCfg.dofValue (5);

      double vf = dx * std::cos (theta)
	+ dy * std::sin (theta);
      double vlat =  dy * std::cos (theta)
	- dx * std::sin (theta);

      // Compute speed vector intersection with the speed cosntraint

      const double tangent = vlat / vf;
	  
      vf = vf >= 0 ?
	1 / sqrt (1 / (attVMaxX * attVMaxX)
		  + (tangent * tangent) / (attVMaxY * attVMaxY))
	: - 1 / sqrt (1 / (attVMinX * attVMinX)
		      + (tangent * tangent) / (attVMaxY * attVMaxY));
      vlat = tangent * vf;
      
      o_speed.push_back (vf);
      o_speed.push_back (vlat);
	  
      // Compute elementary walk time.
      const double dt = integrationStep / sqrt (vf * vf + vlat * vlat);

      return dt; 
    }

    double Distance::distance (const CkwsConfig &i_cfg1,
			       const CkwsConfig &i_cfg2) const
    {
      double deltaX = i_cfg2.dofValue (0) - i_cfg1.dofValue (0);
      double deltaY = i_cfg2.dofValue (1) - i_cfg1.dofValue (1);
      double vecNorm = sqrt (pow (deltaX, 2) + pow (deltaY, 2));

      const unsigned int stepsNb = ceil (vecNorm / attIntegrationStep);
      unsigned int stepIndex = 0;
      double timeDistance = 0.;

      do
	{
	  const double dt = elementaryCost (i_cfg1, i_cfg2, stepsNb, stepIndex);
	  timeDistance += dt;
	  stepIndex++;
	}
      while (stepIndex < stepsNb);
     
      return timeDistance;
    }    

    double Distance::cost (const CkwsDirectPathConstShPtr& i_directPath) const
    {
      return distance (i_directPath->startConfiguration (),
		       i_directPath->endConfiguration ());
	}

    double Distance::cost (const CkwsPathConstShPtr &i_path) const
    {
      const double cost = CkwsDistance::cost (i_path);
	      
      return cost;
    }
    
    double Distance::integrationStep () const
    {
      return attIntegrationStep;
    }
  } // end of namespace hashoptimizer.
} // end of namespace kws.
