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
      else hppDout (info, "Optimizer::init successful");
      
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

    ktStatus Optimizer::doOptimizePath (const CkwsPathShPtr& io_path)
    {
      CkwsAdaptiveShortcutOptimizerShPtr basicOptimizer
	= CkwsAdaptiveShortcutOptimizer::create ();
      basicOptimizer->maxNbLoop (NbOptimizationLoops ());

      CkwsPathShPtr copyPath = CkwsPath::createCopy (io_path);

      if (KD_ERROR == basicOptimizer->optimizePath (copyPath))
	{
	  hppDout(error, "Basic optimization could not be completed");
	  return KD_ERROR;
	}
      
      i_path_ = copyPath;
      o_path_ = CkwsPath::create (device ());
      
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
      for (unsigned int i = 0; i < configsNumber - 1; i++)
	{
	  hppDout (notice, "alignPathConfigs: " << i);

	  // Hash direct path and reorient configurations.
	  if (KD_ERROR == appendHashedDP (i))
	    hppDout(error, "Could not append modified direct path " << i);
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
    appendHashedDP (unsigned int i_int)
    {
      CkwsDirectPathShPtr i_dp 
	= CkwsDirectPath::createCopy (inPath ()->directPath (i_int));

      CkwsConfig dpStartCfg (device ());
      if (i_int == 0)
	dpStartCfg = i_dp->startConfiguration ();
      else outPath ()->getConfigAtEnd (dpStartCfg);
      CkwsConfig dpEndCfg (device ());
      i_dp->getConfigAtEnd (dpEndCfg);

      // Check first if direct path is hashable. If not append direct
      // path by only modifying its end configuration.
      double deltaX = dpEndCfg.dofValue (0) - dpStartCfg.dofValue (0);
      double deltaY = dpEndCfg.dofValue (1) - dpStartCfg.dofValue (1);
      double dpNorm = sqrt (pow (deltaX, 2) + pow (deltaY, 2));
      
      unsigned int nbSteps = (int)(dpNorm / stepSize ());
      hppDout (notice, dpNorm << ", " << stepSize () << ", " << nbSteps); 
      
      if (nbSteps < minStepsNb ())
      	{
      	  // Try to reorient direct path end config.
      	  // reorientDPEndConfig (i_path, i_dpValidator, i_cfgValidator, i_int,
      	  // 		       dpEndCfg);

      	  // Append modified direct path
      	  CkwsSMLinearShPtr linearSM = CkwsSMLinear::create ();
      	  CkwsDirectPathShPtr directPath 
      	    = linearSM->makeDirectPath (dpStartCfg, dpEndCfg);
	  
      	  //FIXME: check direct path validity?
      	  if (KD_ERROR == outPath ()->appendDirectPath (directPath))
      	    {
      	      hppDout (error, "Could not append DP " << i_int);
      	      return KD_ERROR;
      	    }
      	  else 
      	    {
      	      hppDout (notice, "Appended DP " << i_int);
      	      return KD_OK;
      	    }
      	}
      
      // Hash direct path.
      unsigned int i = 0;
      CkwsConfig lastCfg = dpStartCfg;
      CkwsConfig originalCfg (device ());

      // Append all step direct paths except for last one.
      while (i < nbSteps - 1)
	{
	  hppDout (notice, "Appending step direct path " << i
		   << " of " << nbSteps - 1);

	  getOriginalConfig (i_int, i, nbSteps, originalCfg);

	  if (KD_ERROR == appendStepDP (originalCfg, dpEndCfg, i, lastCfg))
	    {
	      hppDout (error, "Could not reorient configuration " << i);
	      return KD_ERROR;
	    }

	  i++;
	}
      
      // Append last direct path.
      hppDout (notice, "Appending step direct path " << i 
	       << " of " << nbSteps - 1);
      if (KD_ERROR == appendLastStepDP (i_int))
	{
	  hppDout (error, "Could not append last direct path.");
	  return KD_ERROR;
	}
      
      return KD_OK;
    }

    ktStatus Optimizer::
    rotateDPEndConfig (unsigned int i_int,
		       CkwsConfig& io_config)
    {
      inPath ()->getConfiguration (i_int + 1, io_config);
      unsigned int configsNumber = inPath ()->countConfigurations (); 

      if (i_int == configsNumber - 2)
	{
	  hppDout (notice, "Cannot rotate path end configuration");
	  return KD_OK;
	}
      else
	{
	  CkwsConfig nextDPEndCfg (device ());
	  inPath ()->getConfiguration (i_int + 2, nextDPEndCfg);

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
    tryOrientFrontalDPEndConfig (unsigned int i_int,
				 CkwsConfig& o_config)
    {
      // Keep the same if it is the direct path end configuration.
      unsigned int nbConfig = inPath ()->countConfigurations ();
      if (i_int == nbConfig - 2)
	{
	  hppDout (notice, "Kept path end configuration.");

	  inPath ()->getConfigAtEnd (o_config);
	  return KD_OK;
	}

      CkwsDirectPathShPtr ithDP
	= CkwsDirectPath::createCopy (inPath ()->directPath (i_int));
      CkwsDirectPathShPtr ithNextDP 
	= CkwsDirectPath::createCopy (inPath ()->directPath (i_int + 1));

      CkwsConfig ithDPStartCfg (device ());
      CkwsConfig ithDPEndCfg (device ());
      CkwsConfig ithNextDPEndCfg (device ());
      inPath ()->getConfiguration (i_int, ithDPStartCfg);
      inPath ()->getConfiguration (i_int + 1, ithDPEndCfg);
      inPath ()->getConfiguration (i_int + 2, ithNextDPEndCfg);
      
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
    tryOrientLateralDPEndConfig (const CkwsConfig& i_originalConfig,
				 unsigned int i_int,
				 CkwsConfig& io_reorientedConfig)
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
      		   << i_int);

	  CkwsConfig lastCfg (device ());
	  outPath ()->getConfigAtEnd (lastCfg);

	  io_reorientedConfig = i_originalConfig;
	  rotateDPEndConfig (i_int, io_reorientedConfig);
	  
	  tryAppendOriginalStepDP (i_originalConfig, i_int, lastCfg,
				   io_reorientedConfig);
      	}
      else
	{
	  // Try to append lateral step direct path.
	  hppDout (notice, "Trying to append lateral step direct path "
		   << i_int);
	  tryAppendLateralLastStepDP (i_originalConfig, i_int,
				      io_reorientedConfig);
	}

      return KD_OK;
    }

    ktStatus Optimizer::
    tryAppendFrontalLastStepDP (const CkwsConfig& i_originalConfig,
				unsigned int i_int,
				CkwsConfig& io_reorientedConfig)
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
	  
	  hppDout (notice, "Trying lateral orientation " << i_int);
	  tryOrientLateralDPEndConfig (i_originalConfig, i_int,
				       io_reorientedConfig);
	  
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
		   << i_int);
	  tryOrientLateralDPEndConfig (i_originalConfig, i_int,
				       io_reorientedConfig);
	}
      else
	{
	  // Append last step direct path.
	  if (KD_ERROR == outPath ()->appendDirectPath (stepDP))
	    {
	      hppDout (error, "Could not append frontal step direct path " 
		       << i_int);
	      return KD_ERROR;
	    }
	}

      return KD_OK;  
    }

    ktStatus Optimizer::
    tryAppendLateralLastStepDP (const CkwsConfig& i_originalConfig,
				unsigned int i_int,
				CkwsConfig& io_reorientedConfig)
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
	      
	      hppDout (warning, " try previous lateral step config " << i_int);

	      if (KD_ERROR ==
		  tryPreviousLateralStepConfig (i_originalConfig,
						i_originalConfig, i_int, lastCfg,
						io_reorientedConfig))
		{
		  hppDout (warning,
			   "Lateral orientation invalid, trying original config "
			   << i_int);

		  io_reorientedConfig = i_originalConfig;
		  rotateDPEndConfig (i_int, io_reorientedConfig);
		  
		  tryAppendOriginalStepDP (i_originalConfig, i_int, lastCfg,
					   io_reorientedConfig);
		}
	    }
	  else
	    {
	      // Append step direct path with lateral orientation.
	      if (KD_ERROR == outPath ()->appendDirectPath (stepDP))
		{
		  hppDout (error,
			   "Could not append last lateral step direct path " 
			   << i_int);
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
		       << i_int);
	      return KD_ERROR;
	    }
	}
      
      return KD_OK;
    }

    ktStatus Optimizer::getOriginalConfig (unsigned int i_dpIndexInt,
					   unsigned int i_stepIndexInt,
					   unsigned int i_stepNumberInt,
					   CkwsConfig& o_config)
    {
      CkwsDirectPathShPtr directPath 
	= CkwsDirectPath::createCopy (inPath ()->directPath (i_dpIndexInt));
      
      if (i_stepIndexInt < i_stepNumberInt - 1)
	{
	  directPath->getConfigAtDistance (i_stepIndexInt * stepSize (),
					   o_config);
	}
      else
	{
	  directPath->getConfigAtEnd (o_config);
	}
      
      return KD_OK;
    }

    ktStatus Optimizer::nextStepConfig (const CkwsConfig& i_originalConfig, 
					const CkwsConfig& i_beginConfig,
					const CkwsConfig& i_endConfig,
					unsigned int i_orientation,
					CkwsConfig& o_config)
    {
      if (i_beginConfig == i_endConfig)
	{
	  hppDout (error,
		   "Begin and end configuration of direct path are the same");
	  return KD_ERROR;
	}

      double step = stepSize ();
      double deltaX = i_endConfig.dofValue (0) - i_beginConfig.dofValue (0);
      double deltaY = i_endConfig.dofValue (1) - i_beginConfig.dofValue (1);
      double vectorNorm = sqrt (pow (deltaX, 2) + pow (deltaY, 2));

      o_config.dofValue (0, i_beginConfig.dofValue (0)
			 + step * deltaX/vectorNorm);
      o_config.dofValue (1, i_beginConfig.dofValue (1)
			 + step * deltaY/vectorNorm);

      if (i_orientation == FRONTAL)
      o_config.dofValue (5, atan2 (deltaY, deltaX));
      else if (i_orientation == LATERAL)
	o_config.dofValue (5, atan2 (deltaY, deltaX) + lateralAngle ());
      else if (i_orientation == LATERAL_ADJUSTED)
	{
	  o_config.dofValue (5, atan2 (deltaY, deltaX) + lateralAngle ());
	}
      else if (i_orientation == ORIGINAL)
	{
	  // FIXME: find a way to retrieve original configuration.
	  adjustOriginalConfig (i_originalConfig, i_endConfig, o_config); 
	}
      
      cfgValidator ()->validate (o_config);

      if (!o_config.isValid ())
	{
	  hppDout (warning, "Intermediate configuration is not valid");
	  return KD_ERROR;
	}
      else return KD_OK;
    }

    ktStatus Optimizer::
    adjustLateralConfig (const CkwsConfig& i_endConfig,
			 const CkwsConfig& i_nextDPEndConfig,
			 CkwsConfig& io_config)
    {
      // FIXME: write body
      return KD_OK;
    }

    ktStatus Optimizer::
    adjustOriginalConfig (const CkwsConfig& i_originalConfig,
			  const CkwsConfig& i_endConfig,
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
    appendStepDP (const CkwsConfig& i_originalConfig,
		  const CkwsConfig& i_dpEndConfig,
		  unsigned int i_int,
		  CkwsConfig& io_lastConfig)
    {
      // Get next step configuration with frontal orientation.
      CkwsConfig nextStepCfg (device ());
      
      if (KD_ERROR == nextStepConfig (i_originalConfig, io_lastConfig,
				      i_dpEndConfig, FRONTAL, nextStepCfg))
	{
	  // Get next step configuratio with lateral orientation.
	  tryLateralStepConfig (i_originalConfig, i_dpEndConfig, i_int,
				io_lastConfig, nextStepCfg);
	}
      else
	{
	  // Try to append step DP with frontal orientation.
	  tryAppendFrontalStepDP (i_originalConfig, i_dpEndConfig, i_int,
				  io_lastConfig, nextStepCfg);
	}

      return KD_OK;
    }

    ktStatus Optimizer::
    tryLateralStepConfig (const CkwsConfig& i_originalConfig,
			  const CkwsConfig& i_dpEndConfig,
			  unsigned int i_int,
			  CkwsConfig& io_lastConfig,
			  CkwsConfig& io_reorientedConfig)
    {
      if (KD_ERROR == nextStepConfig (i_originalConfig, io_lastConfig,
				      i_dpEndConfig, LATERAL_ADJUSTED,
				      io_reorientedConfig))
	{
	  // Get next step configuration with original configuration.
	  tryOriginalStepConfig (i_originalConfig, i_dpEndConfig, i_int,
				 io_lastConfig, io_reorientedConfig);
	}
      else 
	{
	  // Try to append step DP with lateral orientation.
	  tryAppendLateralStepDP (i_originalConfig, i_dpEndConfig, i_int,
				  io_lastConfig, io_reorientedConfig);
	}
      
      return KD_OK;
    }

    ktStatus Optimizer::
    tryOriginalStepConfig (const CkwsConfig& i_originalConfig,
			   const CkwsConfig& i_dpEndConfig,
			   unsigned int i_int,
			   CkwsConfig& io_lastConfig,
			   CkwsConfig& io_reorientedConfig)
    {
      nextStepConfig (i_originalConfig, io_lastConfig, i_dpEndConfig,
		      ORIGINAL, io_reorientedConfig);
      
      // Try to append step DP with original orientation.
      tryAppendOriginalStepDP (i_originalConfig, i_int, io_lastConfig,
			       io_reorientedConfig);
      
      return KD_OK;
    }   

    ktStatus Optimizer::
    tryAppendFrontalStepDP (const CkwsConfig& i_originalConfig,
			    const CkwsConfig& i_dpEndConfig,
			    unsigned int i_int,
			    CkwsConfig& io_lastConfig,
			    CkwsConfig& io_reorientedCfg)
    {
      if (io_lastConfig.isEquivalent (io_reorientedCfg))
	{
	  hppDout (error, "StepDP was not made.");
	  return KD_ERROR;
	}

      // Check if begin and end configurations are facing opposite
      // sides and return error in this case.
      double angleDiff =
	io_reorientedCfg.dofValue (5) - io_lastConfig.dofValue (5);

      if (angleDiff < 0)
	angleDiff += 2 * M_PI; 
	  
      if (angleDiff > 2 * M_PI)
	angleDiff -= 2 * M_PI;
	
      if  (angleDiff == M_PI)
	{
	  hppDout (warning,
		   "Singularity detected, frontal step direct path not valid.");
	  
	  tryLateralStepConfig (i_originalConfig, i_dpEndConfig, i_int,
				io_lastConfig, io_reorientedCfg);
	  
	  return KD_OK;
	}
      
      CkwsSMLinearShPtr linearSM = CkwsSMLinear::create ();
      CkwsDirectPathShPtr stepDP 
	= linearSM->makeDirectPath (io_lastConfig, io_reorientedCfg);
      dpValidator ()->validate (*stepDP);
      
      if (!stepDP->isValid ())
	{
	  // Get next step configuration with lateral orientation.
	  tryLateralStepConfig (i_originalConfig, i_dpEndConfig, i_int,
				io_lastConfig, io_reorientedCfg);
	}
      else
	{
	  // Append step direct path with frontal orientation.
	  if (KD_ERROR == outPath ()->appendDirectPath (stepDP))
	    {
	      hppDout (error, "Could not append frontal step direct path " 
		       << i_int);
	      return KD_ERROR;
	    }
	  io_lastConfig = io_reorientedCfg;
	}

      return KD_OK;
    }

    ktStatus Optimizer::
    tryAppendLateralStepDP (const CkwsConfig& i_originalConfig,
			    const CkwsConfig& i_dpEndConfig,
			    unsigned int i_int,
			    CkwsConfig& io_lastConfig,
			    CkwsConfig& io_reorientedCfg)
    {
      // Check if begin and end configurations are the same.
      if (io_lastConfig.isEquivalent (io_reorientedCfg))
	{
	  hppDout (error, "StepDP was not made.");
	  return KD_ERROR;
	}

      // Check if begin and end configurations are facing opposite
      // sides and rotate end configuration in this case.
      double angleDiff =
	io_reorientedCfg.dofValue (5) - io_lastConfig.dofValue (5);

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
      
      hppDout (notice, "last config: " << incindent << iendl
	       << io_lastConfig.dofValue (0) << iendl
	       << io_lastConfig.dofValue (1) << iendl
	       << io_lastConfig.dofValue (2) << iendl
	       << io_lastConfig.dofValue (3) << iendl
	       << io_lastConfig.dofValue (4) << iendl
	       << io_lastConfig.dofValue (5) << decindent);

      hppDout (notice, "reoriented config: " << incindent << iendl
	       << io_reorientedCfg.dofValue (0) << iendl
	       << io_reorientedCfg.dofValue (1) << iendl
	       << io_reorientedCfg.dofValue (2) << iendl
	       << io_reorientedCfg.dofValue (3) << iendl
	       << io_reorientedCfg.dofValue (4) << iendl
	       << io_reorientedCfg.dofValue (5) << decindent);
      
      CkwsSMLinearShPtr linearSM = CkwsSMLinear::create ();
      CkwsDirectPathShPtr stepDP 
	= linearSM->makeDirectPath (io_lastConfig, io_reorientedCfg);
      dpValidator ()->validate (*stepDP);
      
      if (!stepDP->isValid ())
	{
	  // Flip configuration to try other lateral orientation and
	  // make new direct path.
	  hppDout (warning, "Trying step DP with flipped configuration.");
	  
	  io_reorientedCfg.dofValue (5, io_reorientedCfg.dofValue (5)
				     + M_PI);

	  if (io_lastConfig.isEquivalent (io_reorientedCfg))
	    {
	      hppDout (error, "StepDP was not made.");
	      return KD_ERROR;
	    }

	  CkwsDirectPathShPtr stepDP 
	    = linearSM->makeDirectPath (io_lastConfig, io_reorientedCfg);
	  dpValidator ()->validate (*stepDP);

	  if (!stepDP->isValid ())
	    {
	      // Try to orient previous config lateral and make new direct path.
	      io_reorientedCfg.dofValue (5, io_reorientedCfg.dofValue (5)
					 + M_PI);
	      
	      if (KD_ERROR ==
		  tryPreviousLateralStepConfig (i_originalConfig, i_dpEndConfig,
						i_int, io_lastConfig,
						io_reorientedCfg))
		{
		  hppDout (notice, "Trying original config.");
		  tryOriginalStepConfig (i_originalConfig, i_dpEndConfig,
					 i_int, io_lastConfig, io_reorientedCfg);
		}
	    }
	  else
	    {
	      // Append step direct path with lateral orientation.
	      if (KD_ERROR == outPath ()->appendDirectPath (stepDP))
		{
		  hppDout (error, "Could not append lateral step direct path " 
			   << i_int);
		  return KD_ERROR;
		}
	      io_lastConfig = io_reorientedCfg;
	      lateral_angle_ = - lateral_angle_;
	    }
	}
      else
	{
	  // Append step direct path with lateral orientation.
	  if (KD_ERROR == outPath ()->appendDirectPath (stepDP))
	    {
	      hppDout (error, "Could not append lateral step direct path " 
		       << i_int);
	      return KD_ERROR;
	    }
	  io_lastConfig = io_reorientedCfg;
	}
      
      return KD_OK;
    }
      
    ktStatus Optimizer::
    tryPreviousLateralStepConfig (const CkwsConfig& i_originalConfig,
				  const CkwsConfig& i_dpEndConfig,
				  unsigned int i_int,
				  CkwsConfig& io_lastConfig,
				  CkwsConfig& io_reorientedConfig)
    {
      hppDout (notice, " try previous original step config " << i_int);

      // Remove last step direct path only if it inside the current
      // direct path.
      if (i_int == 0)
	{
	  hppDout (warning, "cannot remove direct path.");
	  return KD_ERROR;
	}
      else
	{
	  outPath ()->extractToDirectPath (outPath ()->countDirectPaths () - 1);
	  outPath ()->getConfigAtEnd (io_lastConfig);
	}
	  
      // Replace it with one where last configuration is oriented
      // laterally.
      CkwsConfig lateralCfg (device ());
      if (KD_ERROR == nextStepConfig (i_originalConfig, io_lastConfig,
				      i_dpEndConfig, LATERAL_ADJUSTED,
				      lateralCfg))
	{
	  // FIXME: What to do in this case.
	  hppDout (error, "Previous lateral configuration not valid.");
	  return KD_OK;
	}
      else
	{
	  hppDout (notice, "Appending previous lateral step DP.");
	  tryAppendLateralStepDP (i_originalConfig, i_dpEndConfig, i_int - 1,
				  io_lastConfig, lateralCfg);
	  
	  hppDout (notice, "Appending lateral step DP.");
	  tryAppendLateralStepDP (i_originalConfig, i_dpEndConfig, i_int,
				  io_lastConfig, io_reorientedConfig);
	}
	
      return KD_OK;
    }

    ktStatus Optimizer::
    tryPreviousOriginalStepConfig (const CkwsConfig& i_originalConfig,
				   const CkwsConfig& i_dpEndConfig,
				   unsigned int i_int,
				   CkwsConfig& io_lastConfig,
				   CkwsConfig& io_reorientedConfig)
    {
      // Remove last step direct path only if it inside the current
      // direct path.
      if (i_int == 0)
	{
	  hppDout (warning, "cannot remove direct path.");
	  return KD_ERROR;
	}
      else
	{
	  outPath ()->extractToDirectPath (outPath ()->countDirectPaths () - 1);
	  outPath ()->getConfigAtEnd (io_lastConfig);
	}
	  
      // Replace it with one where last configuration is the orginal
      // configuration.
      CkwsConfig originalCfg (device ());
      if (KD_ERROR == nextStepConfig (i_originalConfig, io_lastConfig,
				      i_dpEndConfig, ORIGINAL, originalCfg))
	{
	  // This error should not happen as original configuration is
	  // always valid.
	  hppDout (error, "Previous original configuration not valid!!!");
	  return KD_ERROR;
	}
      else
	{
	  hppDout (notice, "Appending previous original step DP.");
	  tryAppendOriginalStepDP (i_originalConfig, i_int - 1, io_lastConfig,
				   originalCfg);
	  
	  hppDout (notice, "Appending original last step DP.");
	  tryAppendOriginalStepDP (i_originalConfig, i_int, io_lastConfig,
				   io_reorientedConfig);
	}
	
      return KD_OK;
    }

    ktStatus Optimizer::
    tryAppendOriginalStepDP (const CkwsConfig& i_originalConfig,
			     unsigned int i_int,
			     CkwsConfig& io_lastConfig,
			     CkwsConfig& io_reorientedCfg)
    {
      // io_reorientedCfg = i_originalConfig;
      // rotateDPEndConfig (i_path, i_int, io_reorientedCfg);
      
      if (io_lastConfig.isEquivalent (io_reorientedCfg))
	{
	  hppDout (error, "StepDP was not made.");
	  return KD_ERROR;
	}

      // Check if begin and end configurations are facing opposite
      // sides and return error in this case.
      double angleDiff =
	io_reorientedCfg.dofValue (5) - io_lastConfig.dofValue (5);
      
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
	= linearSM->makeDirectPath (io_lastConfig, io_reorientedCfg);
      dpValidator ()->validate (*stepDP);
      
      if (!stepDP->isValid ())
	{
	  // Flip configuration to try other original orientation and
	  // make new direct path.
	  hppDout (warning,
		   "Trying step DP with flipped original configuration.");
	  
	  io_reorientedCfg.dofValue (5, io_reorientedCfg.dofValue (5)
				     + M_PI);

	  if (io_lastConfig.isEquivalent (io_reorientedCfg))
	    {
	      hppDout (error, "StepDP was not made.");
	      return KD_ERROR;
	    }

	  CkwsDirectPathShPtr stepDP 
	    = linearSM->makeDirectPath (io_lastConfig, io_reorientedCfg);
	  dpValidator ()->validate (*stepDP);
	  
	  if (!stepDP->isValid ())
	    {
	      // Try original previous config and make new direct path.
	      io_reorientedCfg.dofValue (5, io_reorientedCfg.dofValue (5)
					 + M_PI);
	      
	      hppDout (warning, " try previous original step config " << i_int);

	      if (KD_ERROR ==
		  tryPreviousOriginalStepConfig (i_originalConfig,
						 i_originalConfig, i_int,
						 io_lastConfig,
						 io_reorientedCfg))
		{
		  hppDout (error,
			   "Failed to append original previous step direct path "
			   << i_int);
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
			   << i_int);
		  return KD_ERROR;
		}
	      io_lastConfig = io_reorientedCfg;
	    }
	}
      else
	{
	  // Append step direct path with original orientation.
	  if (KD_ERROR == outPath ()->appendDirectPath (stepDP))
	    {
	      hppDout (error, "Could not append original step direct path " 
		       << i_int);
	      return KD_ERROR;
	    }
	  io_lastConfig = io_reorientedCfg;
	}

      return KD_OK;
    }

    ktStatus Optimizer::
    appendLastStepDP (unsigned int i_int)
    {
      // Try to orient direct path end configuration frontally.
      CkwsConfig dpEndCfg (device ());
      CkwsConfig originalCfg (device ());
      inPath ()->getConfiguration (i_int + 1, originalCfg);

      if (KD_ERROR == tryOrientFrontalDPEndConfig (i_int, dpEndCfg))
	{
	  hppDout (warning,
		   "Frontal orientation failed, trying lateral orientation "
		   << i_int);
	  tryOrientLateralDPEndConfig (originalCfg, i_int, dpEndCfg);
	}
      else
	{
	  hppDout (notice,
		   "Trying to append frontal step direct path "
		   << i_int);
	  tryAppendFrontalLastStepDP (originalCfg, i_int, dpEndCfg);
	}

      return KD_OK;
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
