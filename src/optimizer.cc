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
 * \file src/optimizer.cc
 *
 * \brief Implementation of Optimizer.
 */

#include <iostream>
#include <math.h>

#include <KineoWorks2/kwsValidatorCfgCollision.h>
#include <KineoWorks2/kwsDevice.h>
#include <KineoWorks2/kwsValidatorDPCollision.h>
#include <KineoWorks2/kwsSMLinear.h>
#include <KineoWorks2/kwsAdaptiveShortcutOptimizer.h>

//FIXME: update later hpp-util in robotpkg to include sstream
#include <sstream>
#include <hpp/util/debug.hh>
#include <hpp/util/indent.hh>

#include "kws/hash-optimizer/directpath.hh"
#include "kws/hash-optimizer/steeringmethod.hh"
#include "kws/hash-optimizer/optimizer.hh"

namespace kws
{
  namespace hashoptimizer
  {
    ktStatus Optimizer::init (const OptimizerWkPtr& i_weakPtr)
    {
      if (KD_ERROR == CkwsPathOptimizer::init (i_weakPtr))
      {
	hppDout (error, "Optimizer::init failed");
	return KD_ERROR;
      }
      else 
	{
	  hppDout (info, "Optimizer::init successful");
	}

      return KD_OK;
    }

    OptimizerShPtr Optimizer::create (unsigned int i_nbLoops,
				      double i_double,
				      unsigned int i_nbSteps)
    {
      Optimizer* ptr = new Optimizer (i_nbLoops, i_double, i_nbSteps);
      OptimizerShPtr shPtr (ptr);

      if (ptr->init (shPtr) != KD_OK)
	{
	  shPtr.reset ();
	}

      return shPtr;
    }

    Optimizer::~Optimizer ()
    {
    }
    
    unsigned int Optimizer::NbOptimizationLoops ()
    {
      return max_nb_optimization_loops;
    }

    unsigned int Optimizer::minStepsNb ()
    {
      return min_steps_number_;
    }

    double Optimizer::stepSize ()
    {
      return step_size_;
    }

    double Optimizer::lateralAngle ()
    {
      return lateral_angle_;
    }

    CkwsPathShPtr Optimizer::inPath () const
    {
      return i_path_;
    }

    CkwsPathShPtr Optimizer::outPath ()
    {
      return o_path_;
    }

    CkwsValidatorDPCollisionShPtr Optimizer::dpValidator () const
    {
      return dp_validator_;
    }
    
    CkwsValidatorCfgCollisionShPtr Optimizer::cfgValidator () const
    {
      return cfg_validator_;
    }

    CkwsConfig Optimizer::originalConfig () const
    {
      return *original_config_;
    }

    unsigned int Optimizer::dpIndex () const
    {
      return dp_index_;
    }

    unsigned int Optimizer::stepIndex () const
    {
      return step_index_;
    }

    unsigned int Optimizer::stepsNb () const
    {
      return steps_number_;
    }

    ktStatus Optimizer::doOptimizePath (const CkwsPathShPtr& io_path)
    {
      // Optimize path first with adaptive shortcut optimizer.
      CkwsAdaptiveShortcutOptimizerShPtr basicOptimizer
	= CkwsAdaptiveShortcutOptimizer::create ();
      basicOptimizer->maxNbLoop (NbOptimizationLoops ());

      CkwsPathShPtr copyPath = CkwsPath::createCopy (io_path);

      if (KD_ERROR == basicOptimizer->optimizePath (copyPath))
	{
	  hppDout(error, "Basic optimization could not be completed");
	  return KD_ERROR;
	}
      
      // Rebuild optimized path with direct path that take only
      // translation position variables in interpolation and length
      // computation.
      CkwsConfig dpStartCfg (device ());
      CkwsConfig dpEndCfg (device ());
      CkwsSteeringMethodShPtr steeringMethod = SteeringMethod::create ();
      i_path_ = CkwsPath::create (device ());
      for (unsigned int i = 0; i < copyPath->countConfigurations () - 1; i++)
	{
	  copyPath->getConfiguration (i, dpStartCfg);
	  copyPath->getConfiguration (i + 1, dpEndCfg);

	  CkwsDirectPathShPtr directPath = 
	    steeringMethod->makeDirectPath (dpStartCfg, dpEndCfg);

	  hppDout (notice, directPath->length ());
	  
	  i_path_->appendDirectPath (directPath);
	}

      // Start hash optimization.
      o_path_ = CkwsPath::create (device ());
      CkwsConfig startCfg (device ());
      inPath ()->getConfigAtStart (startCfg);
      outPath ()->setInitialConfig (startCfg);
      original_config_ = CkwsConfig::create (device ());

      if (KD_ERROR == alignPathConfigs ())
	{
	  hppDout(error, "Hash optimization could not be completed");
	}
      
      hppDout (notice, "outPath number of nodes: " 
	       << outPath ()->countConfigurations ());
      *io_path = *outPath ();

      return KD_OK;
    }

    ktStatus Optimizer::alignPathConfigs ()
    {
      // Retrieve collision validators.
      if (KD_ERROR == retrieveValidators ())
	return KD_ERROR;
    
      unsigned int configsNumber = inPath ()->countConfigurations ();
      
      // Go through all direct paths.
      for (dp_index_ = 0; dp_index_ < configsNumber - 1; dp_index_++)
	{
	  hppDout (notice, "alignPathConfigs: " << dp_index_);

	  // Hash direct path and reorient configurations.
	  if (KD_ERROR == appendHashedDP ())
	    {
	      hppDout(error, "Could not append modified direct path "
		      << dpIndex ());
	      return KD_ERROR;
	    }
	}
      
      return KD_OK;
    }

    ktStatus Optimizer::retrieveValidators ()
    {
      ktStatus dpSuccess = KD_OK;
      ktStatus cfgSuccess = KD_OK;

      dp_validator_ = device ()->directPathValidators ()
	->retrieve<CkwsValidatorDPCollision> ();

      if (!dp_validator_)
	{
	  hppDout (warning, "No DP Validator found in device");
	  dp_validator_
	    = CkwsValidatorDPCollision::create (device (),
						inPath ()->maxPenetration (),
						CkwsValidatorDPCollision::
						PROGRESSIVE_FORWARD);
	  if (!dp_validator_)
	    {
	      hppDout (error, "no DP Validator created");
	      dpSuccess = KD_ERROR;
	    }
	}

      dp_validator_
	->algorithm (CkwsValidatorDPCollision::PROGRESSIVE_FORWARD);
      cfg_validator_ = device ()->configValidators ()
	->retrieve<CkwsValidatorCfgCollision> ();

      if (!cfg_validator_)
	{
	  hppDout (warning, "No config Validator found in device");
	  cfg_validator_ = CkwsValidatorCfgCollision::create (device ());
	  if (!cfg_validator_)
	    {
	      hppDout (error, "no config Validator created");
	      cfgSuccess = KD_ERROR;
	    }
	}

      if (dpSuccess != KD_ERROR && cfgSuccess != KD_ERROR)
	return KD_OK;
    }

    ktStatus Optimizer::
    appendHashedDP ()
    {
      CkwsDirectPathShPtr directPath = 
	CkwsDirectPath::createCopy (inPath ()->directPath (dpIndex ()));

      CkwsConfig dpStartCfg (device ());
      if (dpIndex () == 0)
	dpStartCfg = directPath->startConfiguration ();
      else outPath ()->getConfigAtEnd (dpStartCfg);
      CkwsConfig dpEndCfg (device ());
      directPath->getConfigAtEnd (dpEndCfg);

      steps_number_ = (int)(directPath->length () / stepSize ());

      // Check if direct path is too small to be hashed and, set steps
      // number value to append one step, i.e. the whole direct path.
      if (stepsNb () < minStepsNb ())
	steps_number_ = 1;
      
      // Hash direct path.
      step_index_ = 0;
      
      // Append all step direct paths except for last one.
      while (stepIndex () < stepsNb () - 1)
	{
	  hppDout (notice, "Appending step direct path " << stepIndex ()
		   << " of " << stepsNb () - 1);
	  if (KD_ERROR == appendStepDP ())
	    {
	      hppDout (error, "Could not reorient configuration "
		       << stepIndex ());
	      return KD_ERROR;
	    }

	  step_index_++;
	}
      
      // Append last direct path.
      hppDout (notice, "Appending step direct path " << stepIndex ()
	       << " of " << stepsNb () - 1);
      if (KD_ERROR == appendLastStepDP ())
	{
	  hppDout (error, "Could not append last direct path.");
	  return KD_ERROR;
	}
      
      return KD_OK;
    }

    ktStatus Optimizer::
    rotateDPEndConfig (CkwsConfig& io_config)
    {
      inPath ()->getConfiguration (dpIndex () + 1, io_config);
      unsigned int configsNumber = inPath ()->countConfigurations (); 

      if (dpIndex () == configsNumber - 2)
	{
	  hppDout (notice, "Cannot rotate path end configuration");
	  return KD_OK;
	}
      else
	{
	  CkwsConfig nextDPEndCfg (device ());
	  inPath ()->getConfiguration (dpIndex () + 2, nextDPEndCfg);

	  double deltaX = nextDPEndCfg.dofValue (0) - io_config.dofValue (0);
	  double deltaY = nextDPEndCfg.dofValue (1) - io_config.dofValue (1);
	  double angle = io_config.dofValue (5);
	  double scalProd = deltaX * cos (angle) + deltaY * sin (angle);

	  if (scalProd < 0)
	    io_config.dofValue (5, angle + M_PI);
	}
      return KD_OK;
    }

    ktStatus Optimizer::
    tryOrientFrontalDPEndConfig (CkwsConfig& o_config)
    {
      // Keep the same if it is the direct path end configuration.
      unsigned int nbConfig = inPath ()->countConfigurations ();
      if (dpIndex () == nbConfig - 2)
	{
	  hppDout (notice, "Kept path end configuration.");

	  inPath ()->getConfigAtEnd (o_config);
	  return KD_OK;
	}

      CkwsDirectPathShPtr ithDP
	= CkwsDirectPath::createCopy (inPath ()->directPath (dpIndex ()));
      CkwsDirectPathShPtr ithNextDP 
	= CkwsDirectPath::createCopy (inPath ()->directPath (dpIndex () + 1));

      CkwsConfig ithDPStartCfg (device ());
      CkwsConfig ithDPEndCfg (device ());
      CkwsConfig ithNextDPEndCfg (device ());
      inPath ()->getConfiguration (dpIndex (), ithDPStartCfg);
      inPath ()->getConfiguration (dpIndex () + 1, ithDPEndCfg);
      inPath ()->getConfiguration (dpIndex () + 2, ithNextDPEndCfg);
      
      double ithDeltaX = ithDPEndCfg.dofValue (0) - ithDPStartCfg.dofValue (0);
      double ithDeltaY = ithDPEndCfg.dofValue (1) - ithDPStartCfg.dofValue (1);
      double ithNextDeltaX = ithNextDPEndCfg.dofValue (0) 
	- ithDPEndCfg.dofValue (0);
      double ithNextDeltaY = ithNextDPEndCfg.dofValue (1) 
	- ithDPEndCfg.dofValue (1);

      // Try to align direct path end configuration along the average
      // direction of the two adjacent direct paths.
      o_config = ithDPEndCfg;
      o_config.dofValue (5, (atan2 (ithDeltaY, ithDeltaX) 
			     + atan2 (ithNextDeltaY, ithNextDeltaX)) / 2);
      cfgValidator ()->validate (o_config);

      if (!o_config.isValid ())
	  return KD_ERROR;

      return KD_OK;
    }

    ktStatus Optimizer::
    tryOrientLateralDPEndConfig (CkwsConfig& io_reorientedConfig)
    {
      io_reorientedConfig.dofValue (5, io_reorientedConfig.dofValue (5) 
       				    + lateralAngle ());
      cfgValidator ()->validate (io_reorientedConfig);
      
      if (!io_reorientedConfig.isValid ())
      	{
	  // Try to append original step direct path.
	  hppDout (warning,
      		   "Lateral orientation failed, "
		   "Trying to append original step direct path "
		   << dpIndex ());

	  io_reorientedConfig = originalConfig ();
	  rotateDPEndConfig (io_reorientedConfig);
	  
	  tryAppendOriginalStepDP (io_reorientedConfig);
      	}
      else
	{
	  // Try to append lateral step direct path.
	  hppDout (notice, "Trying to append lateral step direct path "
		   << dpIndex ());
	  tryAppendLateralStepDP (io_reorientedConfig);
	}

      return KD_OK;
    }

    ktStatus Optimizer::
    tryAppendFrontalLastStepDP (CkwsConfig& io_reorientedConfig)
    {
      CkwsConfig lastCfg (device ());
      outPath ()->getConfigAtEnd (lastCfg);

      if (lastCfg.isEquivalent (io_reorientedConfig))
	{
	  hppDout (error, "StepDP was not made.");
	  return KD_ERROR;
	}
      
      // Check if begin and end configurations are facing opposite
      // sides and return error in this case.
      double angleDiff =
	io_reorientedConfig.dofValue (5) - lastCfg.dofValue (5);

      if (angleDiff < 0)
	angleDiff += 2 * M_PI; 
	  
      if (angleDiff > 2 * M_PI)
	angleDiff -= 2 * M_PI;
	
      if  (angleDiff == M_PI)
	{
	  hppDout (warning,
		   "Singularity detected, frontal step direct path not valid.");
	  
	  hppDout (notice, "Trying lateral orientation " << dpIndex ());
	  tryOrientLateralDPEndConfig (io_reorientedConfig);
	  
	  return KD_OK;
	}

      CkwsSMLinearShPtr linearSM = CkwsSMLinear::create ();
      CkwsDirectPathShPtr stepDP 
	= linearSM->makeDirectPath (lastCfg, io_reorientedConfig);
      dpValidator ()->validate (*stepDP);
      
      if (!stepDP->isValid ())
	{
	  // Try lateral orientation for direct path end configuration.
	  hppDout (warning,
		   " Step direct path not valid, trying lateral orientation"
		   << dpIndex ());
	  tryOrientLateralDPEndConfig (io_reorientedConfig);
	}
      else
	{
	  // Append last step direct path.
	  if (KD_ERROR == outPath ()->appendDirectPath (stepDP))
	    {
	      hppDout (error, "Could not append frontal step direct path " 
		       << dpIndex ());
	      return KD_ERROR;
	    }
	}

      return KD_OK;
    }

    ktStatus Optimizer::
    tryAppendLateralLastStepDP (CkwsConfig& io_reorientedConfig)
    {
      CkwsConfig lastCfg (device ());
      outPath ()->getConfigAtEnd (lastCfg);

      if (lastCfg.isEquivalent (io_reorientedConfig))
	{
	  hppDout (error, "StepDP was not made.");
	  return KD_ERROR;
	}
      
      // Check if begin and end configurations are facing opposite
      // sides and return error in this case.
      double angleDiff =
	io_reorientedConfig.dofValue (5) - lastCfg.dofValue (5);

      if (angleDiff < 0)
	angleDiff += 2 * M_PI; 
	  
      if (angleDiff > 2 * M_PI)
	angleDiff -= 2 * M_PI;
	
      if  (angleDiff == M_PI)
	{
	  hppDout (warning,
		   "Singularity detected, lateral step direct path not valid.");
	  
	  io_reorientedConfig.dofValue (5, io_reorientedConfig.dofValue (5) 
					- M_PI);
	}

      CkwsSMLinearShPtr linearSM = CkwsSMLinear::create ();
      CkwsDirectPathShPtr stepDP 
	= linearSM->makeDirectPath (lastCfg, io_reorientedConfig);
      dpValidator ()->validate (*stepDP);
      
      if (!stepDP->isValid ())
	{
	  // Flip configuration to try other lateral orientation and
	  // make new direct path.
	  hppDout (warning, "Trying step DP with flipped configuration.");
	  
	  io_reorientedConfig.dofValue (5, io_reorientedConfig.dofValue (5)
					+ M_PI);

	  if (lastCfg.isEquivalent (io_reorientedConfig))
	    {
	      hppDout (error, "StepDP was not made.");
	      return KD_ERROR;
	    }

	  CkwsDirectPathShPtr stepDP 
	    = linearSM->makeDirectPath (lastCfg, io_reorientedConfig);
	  dpValidator ()->validate (*stepDP);

	  if (!stepDP->isValid ())
	    {
	      // Try to orient previous config lateral and make new direct path.
	      io_reorientedConfig.dofValue (5, io_reorientedConfig.dofValue (5)
					    + M_PI);
	      
	      hppDout (warning, " try previous lateral step config "
		       << dpIndex ());

	      if (KD_ERROR ==
		  tryPreviousLateralStepConfig (io_reorientedConfig))
		{
		  hppDout (warning,
			   "Lateral orientation invalid, trying original config "
			   << dpIndex ());

		  io_reorientedConfig = originalConfig ();
		  rotateDPEndConfig (io_reorientedConfig);
		  
		  tryAppendOriginalStepDP (io_reorientedConfig);
		}
	    }
	  else
	    {
	      // Append step direct path with lateral orientation.
	      if (KD_ERROR == outPath ()->appendDirectPath (stepDP))
		{
		  hppDout (error,
			   "Could not append last lateral step direct path " 
			   << dpIndex ());
		  return KD_ERROR;
		}
	      lateral_angle_ = - lateral_angle_;
	    }
	}
      else
	{
	  // Append step direct path with lateral orientation.
	  if (KD_ERROR == outPath ()->appendDirectPath (stepDP))
	    {
	      hppDout (error, "Could not append last lateral step direct path "
		       << dpIndex ());
	      return KD_ERROR;
	    }
	}
      
      return KD_OK;
    }

    ktStatus Optimizer::nextStepConfig (unsigned int i_orientation,
					CkwsConfig& o_config)
    {
      CkwsConfig lastCfg (device ());
      outPath ()->getConfigAtEnd (lastCfg);
      
      CkwsConfig dpEndCfg (device ());
      inPath ()->getConfiguration (dpIndex () + 1, dpEndCfg);

      if (lastCfg == dpEndCfg)
	{
	  hppDout (error,
		   "Begin and end configuration of direct path are the same");
	  return KD_ERROR;
	}
      
      double step = stepSize ();
      double deltaX = dpEndCfg.dofValue (0) - lastCfg.dofValue (0);
      double deltaY = dpEndCfg.dofValue (1) - lastCfg.dofValue (1);
      double vectorNorm = sqrt (pow (deltaX, 2) + pow (deltaY, 2));

      o_config.dofValue (0, lastCfg.dofValue (0)
			 + step * deltaX/vectorNorm);
      o_config.dofValue (1, lastCfg.dofValue (1)
			 + step * deltaY/vectorNorm);

      if (i_orientation == FRONTAL)
      o_config.dofValue (5, atan2 (deltaY, deltaX));
      else if (i_orientation == LATERAL)
	o_config.dofValue (5, atan2 (deltaY, deltaX) + lateralAngle ());
      
      cfgValidator ()->validate (o_config);

      if (!o_config.isValid ())
	{
	  hppDout (warning, "Intermediate configuration is not valid");
	  return KD_ERROR;
	}
      else return KD_OK;
    }

    ktStatus Optimizer::getOriginalConfig (CkwsConfig& o_config)
    {
      CkwsDirectPathShPtr directPath 
	= CkwsDirectPath::createCopy (inPath ()->directPath (dpIndex ()));

      if (stepIndex () < stepsNb () - 1)
	{
	  directPath->getConfigAtDistance ((stepIndex () + 1) * stepSize (),
					   o_config);
	}
      else
	{
	  directPath->getConfigAtEnd (o_config);
	}

      return KD_OK;
    }
    
    ktStatus Optimizer::
    adjustOriginalConfig (const CkwsConfig& i_endConfig,
			  CkwsConfig& io_config)
    {
      // FIXME: write body
      return KD_OK;
    }
		 
    ktStatus Optimizer::
    tryMakeStepDP (const CkwsConfig& i_beginConfig,
		   const CkwsConfig& i_endConfig)
    {
      if (i_beginConfig.isEquivalent (i_endConfig))
	{
	  hppDout (error, "StepDP was not made.");
	  return KD_ERROR;
	}

      CkwsSMLinearShPtr linearSM = CkwsSMLinear::create ();
      CkwsDirectPathShPtr stepDP 
	= linearSM->makeDirectPath (i_beginConfig, i_endConfig);
      dpValidator ()->validate (*stepDP);
      
      if (!stepDP->isValid ())
	{
	  hppDout (error, "Step DP is not valid");
	  return KD_ERROR;
	}
      else return KD_OK;
    }

    ktStatus Optimizer::
    appendStepDP ()
    {
      // Set original configuration value.
      CkwsConfig originalCfg(device ());
      getOriginalConfig (originalCfg);
      *original_config_ = originalCfg;

      // Get next step configuration with frontal orientation.
      CkwsConfig nextStepCfg (device ());

      if (KD_ERROR == nextStepConfig (FRONTAL, nextStepCfg))
	{
	  // Get next step configuratio with lateral orientation.
	  return tryLateralStepConfig (nextStepCfg);
	}
      else
	{
	  // Try to append step DP with frontal orientation.
	  return tryAppendFrontalStepDP (nextStepCfg);
	}
    }

    ktStatus Optimizer::
    tryLateralStepConfig (CkwsConfig& io_reorientedConfig)
    {
      if (KD_ERROR == nextStepConfig (LATERAL, io_reorientedConfig))
	{
	  // Get next step configuration with original configuration.
	  return tryOriginalStepConfig (io_reorientedConfig);
	}
      else 
	{
	  // Try to append step DP with lateral orientation.
	  return tryAppendLateralStepDP (io_reorientedConfig);
	}
    }

    ktStatus Optimizer::
    tryOriginalStepConfig (CkwsConfig& io_reorientedConfig)
    {
      io_reorientedConfig = originalConfig ();

      // Try to append step DP with original orientation.
      return tryAppendOriginalStepDP (io_reorientedConfig);
    }   

    ktStatus Optimizer::
    tryAppendFrontalStepDP (CkwsConfig& io_reorientedCfg)
    {
      CkwsConfig lastCfg (device ());
      outPath ()->getConfigAtEnd (lastCfg);

      if (lastCfg.isEquivalent (io_reorientedCfg))
	{
	  hppDout (error, "StepDP was not made.");
	  return KD_ERROR;
	}

      // Check if begin and end configurations are facing opposite
      // sides and return error in this case.
      double angleDiff =
	io_reorientedCfg.dofValue (5) - lastCfg.dofValue (5);

      if (angleDiff < 0)
	angleDiff += 2 * M_PI; 
	  
      if (angleDiff > 2 * M_PI)
	angleDiff -= 2 * M_PI;
	
      if  (angleDiff == M_PI)
	{
	  hppDout (warning,
		   "Singularity detected, frontal step direct path not valid.");
	  
	  if (dpIndex () == inPath ()->countConfigurations  () - 2
	      && stepIndex () == stepsNb () - 1)
	    {
	      hppDout (notice, "Keep path end configuration");
	      return tryPreviousLateralStepConfig (io_reorientedCfg);
	    }
	  else return tryLateralStepConfig (io_reorientedCfg);
	}
      
      CkwsSMLinearShPtr linearSM = CkwsSMLinear::create ();
      CkwsDirectPathShPtr stepDP 
	= linearSM->makeDirectPath (lastCfg, io_reorientedCfg);
      dpValidator ()->validate (*stepDP);
      
      if (!stepDP->isValid ())
	{
	  // Get next step configuration with lateral orientation.
	  return tryLateralStepConfig (io_reorientedCfg);
	}
      else
	{
	  // Append step direct path with frontal orientation.
	  if (KD_ERROR == outPath ()->appendDirectPath (stepDP))
	    {
	      hppDout (error, "Could not append frontal step direct path " 
		       << stepIndex ());
	      return KD_ERROR;
	    }

	  lastCfg = io_reorientedCfg;

	  return KD_OK;
	}
    }

    ktStatus Optimizer::
    tryAppendLateralStepDP (CkwsConfig& io_reorientedCfg)
    {
      // Check if begin and end configurations are the same.
      CkwsConfig lastCfg (device ());
      outPath ()->getConfigAtEnd (lastCfg);

      if (lastCfg.isEquivalent (io_reorientedCfg))
	{
	  hppDout (error, "StepDP was not made.");
	  return KD_ERROR;
	}

      // Check if begin and end configurations are facing opposite
      // sides and rotate end configuration in this case.
      double angleDiff =
	io_reorientedCfg.dofValue (5) - lastCfg.dofValue (5);

      if (angleDiff < 0)
	angleDiff += 2 * M_PI; 
	  
      if (angleDiff > 2 * M_PI)
	angleDiff -= 2 * M_PI;
	
      if  (angleDiff == M_PI)
	{
	  hppDout (warning,
		   "Singularity detected, rotating end configuration.");
	  io_reorientedCfg.dofValue (5, io_reorientedCfg.dofValue (5) 
				     - M_PI);
	}
      
      CkwsSMLinearShPtr linearSM = CkwsSMLinear::create ();
      CkwsDirectPathShPtr stepDP 
	= linearSM->makeDirectPath (lastCfg, io_reorientedCfg);
      dpValidator ()->validate (*stepDP);
      
      if (!stepDP->isValid ())
	{
	  // Flip configuration to try other lateral orientation and
	  // make new direct path.
	  hppDout (warning, "Trying step DP with flipped configuration.");
	  
	  io_reorientedCfg.dofValue (5, io_reorientedCfg.dofValue (5)
				     + M_PI);

	  if (lastCfg.isEquivalent (io_reorientedCfg))
	    {
	      hppDout (error, "StepDP was not made.");
	      return KD_ERROR;
	    }

	  CkwsDirectPathShPtr stepDP 
	    = linearSM->makeDirectPath (lastCfg, io_reorientedCfg);
	  dpValidator ()->validate (*stepDP);

	  if (!stepDP->isValid ())
	    {
	      // Try to orient previous config lateral and make new direct path.
	      io_reorientedCfg.dofValue (5, io_reorientedCfg.dofValue (5)
					 + M_PI);
	      
	      if (KD_ERROR ==
		  tryPreviousLateralStepConfig (io_reorientedCfg))
		{
		  hppDout (notice, "Trying original config.");
		  return tryOriginalStepConfig (io_reorientedCfg);
		}
	    }
	  else
	    {
	      // Append step direct path with lateral orientation.
	      if (KD_ERROR == outPath ()->appendDirectPath (stepDP))
		{
		  hppDout (error, "Could not append lateral step direct path " 
			   << stepIndex ());
		  return KD_ERROR;
		}

	      lateral_angle_ = - lateral_angle_;
	      
	      return KD_OK;
	    }
	}
      else
	{
	  // Append step direct path with lateral orientation.
	  if (KD_ERROR == outPath ()->appendDirectPath (stepDP))
	    {
	      hppDout (error, "Could not append lateral step direct path " 
		       << stepIndex ());
	      return KD_ERROR;
	    }
	}
    }
      
    ktStatus Optimizer::
    tryPreviousLateralStepConfig (CkwsConfig& io_reorientedConfig)
    {
      hppDout (notice, " try previous original step config " << stepIndex ());

      // Remove last step direct path only if it inside the current
      // direct path.
      CkwsConfig lastCfg (device ());
      
      if (stepIndex () == 0)
	{
	  hppDout (warning, "cannot remove direct path.");
	  return KD_ERROR;
	}
      else
	{
	  outPath ()->extractToDirectPath (outPath ()->countDirectPaths () - 1);
	  outPath ()->getConfigAtEnd (lastCfg);
	}
	  
      // Replace it with one where last configuration is oriented
      // laterally.
      CkwsConfig lateralCfg (device ());
      if (KD_ERROR == nextStepConfig (LATERAL, lateralCfg))
	{
	  // FIXME: What to do in this case.
	  hppDout (error, "Previous lateral configuration not valid.");
	  return KD_OK;
	}
      else
	{
	  hppDout (notice, "Appending previous lateral step DP.");
	  step_index_--;

	  if (KD_ERROR == tryAppendLateralStepDP (lateralCfg))
	    return KD_ERROR;

	  step_index_++;

	  hppDout (notice, "Appending lateral step DP.");
	  return tryAppendLateralStepDP (io_reorientedConfig);
	}
    }

    ktStatus Optimizer::
    tryPreviousOriginalStepConfig (CkwsConfig& io_reorientedConfig)
    {
      CkwsConfig originalCfg (device ());

      // What to do if start of current direct path is reached
      if (stepIndex () == 0)
	{
	  // Don't do anything if the path start configuration is reached.
	  if (dpIndex () == 0)
	    {
	      hppDout (error,
		       "Reached start of path, cannot remove direct path.");
	      return KD_ERROR;
	    }
	  // Otherwise crossover to previous step direct path and
	  // remove step direct path.
	  else
	    {
	      hppDout (notice, "Crossing to previous direct path");
	      dp_index_--;

	      // Compute previous direct path number of steps.
	      CkwsDirectPathShPtr directPath = 
		CkwsDirectPath::createCopy (inPath ()->directPath (dpIndex ()));
	      steps_number_ = (int)(directPath->length () / stepSize ());

	      // Remove last step direct path from previous direct
	      // path.
	      outPath ()->extractToDirectPath (outPath ()->
					       countDirectPaths () - 1);
	      step_index_ = stepsNb () - 2;
	      getOriginalConfig (originalCfg);
	      *original_config_ = originalCfg;
	    }
	}
      // Remove last appended step direct path.
      else
	{
	  outPath ()->extractToDirectPath (outPath ()->countDirectPaths () - 1);
	  step_index_--;
	  getOriginalConfig (originalCfg);
	  *original_config_ = originalCfg;
	}

      // Replace it with one where last configuration is the orginal
      // configuration.
            
      hppDout (notice, "Appending previous original step DP.");

      if (KD_ERROR == tryAppendOriginalStepDP (originalCfg))
	return KD_ERROR;

      step_index_++;
      // Check if end of direct path has been reached.
      if (stepIndex () == stepsNb () - 2)
	{
	  step_index_ = 0;
	  dp_index_++;

	  // Compute next direct path number of steps.
	  CkwsDirectPathShPtr directPath = 
	    CkwsDirectPath::createCopy (inPath ()->directPath (dpIndex ()));
	  steps_number_ = (int)(directPath->length () / stepSize ());
	}

      getOriginalConfig (originalCfg);
      *original_config_ = originalCfg;

      hppDout (notice, "Appending original last step DP.");
      return tryAppendOriginalStepDP (io_reorientedConfig);
    }

    ktStatus Optimizer::
    tryAppendOriginalStepDP (CkwsConfig& io_reorientedCfg)
    {
      CkwsConfig lastCfg (device ());
      outPath ()->getConfigAtEnd (lastCfg);
      
      if (lastCfg.isEquivalent (io_reorientedCfg))
	{
	  hppDout (error, "StepDP was not made.");
	  return KD_ERROR;
	}

      // Check if begin and end configurations are facing opposite
      // sides and return error in this case.
      double angleDiff =
	io_reorientedCfg.dofValue (5) - lastCfg.dofValue (5);
      
      if (angleDiff < 0)
	angleDiff += 2 * M_PI; 
      
      if (angleDiff > 2 * M_PI)
	angleDiff -= 2 * M_PI;
      
      if  (angleDiff == M_PI)
	{
	  hppDout (warning,
		   "Singularity detected, original step direct path not valid.");
	  
	  io_reorientedCfg.dofValue (5, io_reorientedCfg.dofValue (5) 
				     - M_PI);
	}

      CkwsSMLinearShPtr linearSM = CkwsSMLinear::create ();
      CkwsDirectPathShPtr stepDP 
	= linearSM->makeDirectPath (lastCfg, io_reorientedCfg);
      dpValidator ()->validate (*stepDP);
      
      if (!stepDP->isValid ())
	{
	  if (dpIndex () == inPath ()->countConfigurations  () - 2
	      && stepIndex () == stepsNb () - 1)
	    {
	      hppDout (notice, "keep end configuration");
	      hppDout (notice, "try previous original step config "
		       << stepIndex ());

	      if (KD_ERROR ==
		  tryPreviousOriginalStepConfig (io_reorientedCfg))
		{
		  hppDout (error,
			   "Failed to append original previous step direct path "
			   << stepIndex ());
		  return KD_ERROR;
		}
	    }
	  // Flip configuration to try other original orientation and
	  // make new direct path.
	  hppDout (warning,
		   "Trying step DP with flipped original configuration.");
	  
	  io_reorientedCfg.dofValue (5, io_reorientedCfg.dofValue (5)
				     + M_PI);

	  if (lastCfg.isEquivalent (io_reorientedCfg))
	    {
	      hppDout (error, "StepDP was not made.");
	      return KD_ERROR;
	    }

	  CkwsDirectPathShPtr stepDP 
	    = linearSM->makeDirectPath (lastCfg, io_reorientedCfg);
	  dpValidator ()->validate (*stepDP);
	  
	  if (!stepDP->isValid ())
	    {
	      // Try original previous config and make new direct path.
	      io_reorientedCfg.dofValue (5, io_reorientedCfg.dofValue (5)
					 + M_PI);
	      
	      hppDout (warning, " try previous original step config "
		       << stepIndex ());

	      if (KD_ERROR ==
		  tryPreviousOriginalStepConfig (io_reorientedCfg))
		{
		  hppDout (error,
			   "Failed to append original previous step direct path "
			   << stepIndex ());
		  return KD_ERROR;
		}
	    }
	  else
	    {
	      // Append step direct path with original orientation.
	      if (KD_ERROR == outPath ()->appendDirectPath (stepDP))
		{
		  hppDout (error,
			   "Could not append last original step direct path " 
			   << stepIndex ());
		  return KD_ERROR;
		}
	    }
	}
      else
	{
	  // Append step direct path with original orientation.
	  if (KD_ERROR == outPath ()->appendDirectPath (stepDP))
	    {
	      hppDout (error, "Could not append original step direct path " 
		       << stepIndex ());
	      return KD_ERROR;
	    }
	}

      return KD_OK;
    }

    ktStatus Optimizer::
    appendLastStepDP ()
    {
      // Try to orient direct path end configuration frontally.
      CkwsConfig dpEndCfg (device ());

      if (KD_ERROR == tryOrientFrontalDPEndConfig (dpEndCfg))
	{
	  hppDout (warning,
		   "Frontal orientation failed, trying lateral orientation "
		   << dpIndex ());
	  return tryOrientLateralDPEndConfig (dpEndCfg);
	}
      else
	{
	  hppDout (notice,
		   "Trying to append frontal step direct path "
		   << dpIndex ());
	  return tryAppendFrontalStepDP (dpEndCfg);
	}
    }

    Optimizer::Optimizer (unsigned int i_nbLoops,
			  double i_double,
			  unsigned int i_nbSteps) : CkwsPathOptimizer ()
    {
      max_nb_optimization_loops = i_nbLoops;
      human_size_ = i_double;
      min_steps_number_ = i_nbSteps;
      step_size_ = human_size_/6;
      lateral_angle_ = M_PI / 2;
    }

  } // end of namespace hashoptimizer.
} // end of namespace kws.
