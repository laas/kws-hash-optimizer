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

      CkwsPathShPtr outPath = CkwsPath::create (device ());
      
      if (KD_ERROR == alignPathConfigs (copyPath, outPath))
	{
	  hppDout(error, "Hash optimization could not be completed");
	}
      
      hppDout (notice, "outPath number of nodes: " 
	       << outPath->countConfigurations ());
      *io_path = *outPath;

      return KD_OK;
    }

    ktStatus Optimizer::alignPathConfigs (const CkwsPathShPtr& i_path,
					       CkwsPathShPtr& o_path)
    {
      // Retrieve collision validators.
      CkwsValidatorDPCollisionShPtr dpValidator;
      CkwsValidatorCfgCollisionShPtr cfgValidator;
      if (KD_ERROR == retrieveValidators (i_path, dpValidator, cfgValidator))
	return KD_ERROR;
    
      unsigned int configsNumber = i_path->countConfigurations ();
      CkwsPathShPtr outPath = CkwsPath::create (device ());

      // Go through all direct paths.
      for (unsigned int i = 0; i < configsNumber - 1; i++)
	{
	  hppDout (notice, "alignPathConfigs: " << i);

	  // Hash direct path and reorient configurations.
	  if (KD_ERROR == appendHashedDP (i_path, dpValidator, cfgValidator,
					    i, outPath))
	    hppDout(error, "Could not append modified direct path " << i);
	}
      
      o_path = CkwsPath::create (device ());
      *o_path = *outPath;

      return KD_OK;
    }

    ktStatus Optimizer::retrieveValidators
    (const CkwsPathShPtr& i_path,
     CkwsValidatorDPCollisionShPtr& o_dpValidator,
     CkwsValidatorCfgCollisionShPtr& o_cfgValidator)
    {
      ktStatus dpSuccess = KD_OK;
      ktStatus cfgSuccess = KD_OK;

      o_dpValidator = device ()->directPathValidators ()
	->retrieve<CkwsValidatorDPCollision> ();

      if (!o_dpValidator)
	{
	  hppDout (warning, "No DP Validator found in device");
	  o_dpValidator
	    = CkwsValidatorDPCollision::create (device (),
						i_path->maxPenetration (),
						CkwsValidatorDPCollision::
						PROGRESSIVE_FORWARD);
	  if (!o_dpValidator)
	    {
	      hppDout (error, "no DP Validator created");
	      dpSuccess = KD_ERROR;
	    }
	}

      o_dpValidator
	->algorithm (CkwsValidatorDPCollision::PROGRESSIVE_FORWARD);
      o_cfgValidator = device ()->configValidators ()
	->retrieve<CkwsValidatorCfgCollision> ();

      if (!o_cfgValidator)
	{
	  hppDout (warning, "No config Validator found in device");
	  o_cfgValidator = CkwsValidatorCfgCollision::create (device ());
	  if (!o_cfgValidator)
	    {
	      hppDout (error, "no config Validator created");
	      cfgSuccess = KD_ERROR;
	    }
	}

      if (dpSuccess != KD_ERROR && cfgSuccess != KD_ERROR)
	return KD_OK;
    }

    ktStatus Optimizer::
    appendHashedDP (const CkwsPathShPtr& i_path,
		    const CkwsValidatorDPCollisionShPtr& i_dpValidator,
		    const CkwsValidatorCfgCollisionShPtr& i_cfgValidator,
		    unsigned int i_int,
		    CkwsPathShPtr& io_path)
    {
      CkwsDirectPathShPtr i_dp 
      = CkwsDirectPath::createCopy (i_path->directPath (i_int));

      CkwsConfig dpStartCfg (device ());
      if (i_int == 0)
	dpStartCfg = i_dp->startConfiguration ();
      else io_path->getConfigAtEnd (dpStartCfg);
      CkwsConfig dpEndCfg (device ());
      i_dp->getConfigAtEnd (dpEndCfg);

      // Check first if direct path is hashable. If not append direct
      // path by only modifying its end configuration.
      double deltaX = dpEndCfg.dofValue (0) - dpStartCfg.dofValue (0);
      double deltaY = dpEndCfg.dofValue (1) - dpStartCfg.dofValue (1);
      double dpNorm = sqrt (pow (deltaX, 2) + pow (deltaY, 2));
      
      unsigned int nbSteps = (int)(dpNorm / stepSize ());
      if (nbSteps < minStepsNb ())
	{
	  // Keep original direct path end configuration modulo Pi to
	  // make it point towards the end of the next direct path.
	  rotateDPEndConfig (i_path, i_int, dpEndCfg);

	  // Append modified direct path
	  CkwsSMLinearShPtr linearSM = CkwsSMLinear::create ();
	  CkwsDirectPathShPtr directPath 
	    = linearSM->makeDirectPath (dpStartCfg, dpEndCfg);
	  
	  //FIXME: check direct path validity?
	  if (KD_ERROR == io_path->appendDirectPath (directPath))
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
      
      // Reorient direct path end configuration.
      hppDout (notice, "Reorienting direct path end configuration " 
	       << i_int);

      if (KD_ERROR == reorientDPEndConfig (i_path, i_dpValidator,
					   i_cfgValidator, i_int, dpEndCfg))
	{
	  hppDout(error, "Could not align direct path end configuration "
		  << i_int);
	  return KD_ERROR;
	}

      // Hash direct path.
      unsigned int i = 0;
      CkwsConfig lastCfg = dpStartCfg;
      CkwsConfig nextDPEndCfg (device ());
      i_path->getConfiguration (i_int + 2, nextDPEndCfg);
      CkwsConfig originalCfg (device ());

      while (i < nbSteps)
	{
	  hppDout (notice, "Appending step direct path " << i);

	  // Reorient next configuration and append step direct path

	  getOriginalConfig (i_path, i_int, i, nbSteps, originalCfg);

	  if (KD_ERROR == appendStepDP (originalCfg, dpEndCfg, nextDPEndCfg,
					i_dpValidator, i_cfgValidator, i,
					nbSteps, lastCfg, io_path))
	    {
	      hppDout (error, "Could not reorient configuration " << i);
	      return KD_ERROR;
	    }
	  
	  i++;
	}
      
      return KD_OK;
    }

    ktStatus Optimizer::
    rotateDPEndConfig (const CkwsPathShPtr& i_path,
		       unsigned int i_int,
		       CkwsConfig& io_config)
    {
      i_path->getConfiguration (i_int + 1, io_config);
      unsigned int configsNumber = i_path->countConfigurations (); 

      if (i_int == configsNumber - 2)
	{
	  hppDout (notice, "Cannot rotate path end configuration");
	  return KD_OK;
	}
      else
	{
	  CkwsConfig nextDPEndCfg (device ());
	  i_path->getConfiguration (i_int + 2, nextDPEndCfg);

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
    reorientDPEndConfig (const CkwsPathShPtr& i_path,
			 const CkwsValidatorDPCollisionShPtr& i_dpValidator,
			 const CkwsValidatorCfgCollisionShPtr& i_cfgValidator,
			 unsigned int i_int,
			 CkwsConfig& o_config)
    {
      // Keep the same if it is the direct path end configuration.
      unsigned int nbConfig = i_path->countConfigurations ();
      if (i_int == nbConfig - 2)
	{
	  hppDout (notice, "Kept path end configuration.");

	  i_path->getConfigAtEnd (o_config);
	  return KD_OK;
	}

      CkwsDirectPathShPtr ithDP
	= CkwsDirectPath::createCopy (i_path->directPath (i_int));
      CkwsDirectPathShPtr ithNextDP 
	= CkwsDirectPath::createCopy (i_path->directPath (i_int + 1));

      unsigned int ithNextNbSteps = (int)(ithNextDP->length () / stepSize ());

      // Simply rotate by k*PI if next direct path is too small. 
      if (ithNextNbSteps < minStepsNb ())
	{
	  hppDout (notice, "Rotating direct path end configuration by k*PI" 
		   << i_int);
	  
	  rotateDPEndConfig (i_path, i_int, o_config);
	  
	  return KD_OK;
	}

      CkwsConfig ithDPStartCfg (device ());
      CkwsConfig ithDPEndCfg (device ());
      CkwsConfig ithNextDPEndCfg (device ());
      i_path->getConfiguration (i_int, ithDPStartCfg);
      i_path->getConfiguration (i_int + 1, ithDPEndCfg);
      i_path->getConfiguration (i_int + 2, ithNextDPEndCfg);
      
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
      i_cfgValidator->validate (o_config);

      if (!o_config.isValid ())
	{
	  hppDout (warning, "reorient direct path end config frontally failed "
		   << i_int);
	
	  // Try to reorient configuration orthogonally.
	  tryOrthogonalDPEndConfig (i_path, ithDPEndCfg, ithNextDPEndCfg,
				    i_dpValidator, i_cfgValidator, i_int,
				    o_config);
	}
      else
	{
	  // Verify step direct path after direct path end configuration.
	  tryMakeFrontalDPForEndConfig (i_path, ithDPEndCfg, ithNextDPEndCfg,
					i_dpValidator, i_cfgValidator, i_int,
					o_config);
	}

      return KD_OK;
    }

    ktStatus Optimizer::
    tryOrthogonalDPEndConfig (const CkwsPathShPtr& i_path,
			      const CkwsConfig& i_originalConfig,
			      const CkwsConfig& i_nextDPEndConfig,
			      const CkwsValidatorDPCollisionShPtr&
			      i_dpValidator,
			      const CkwsValidatorCfgCollisionShPtr&
			      i_cfgValidator,
			      unsigned int i_int,
			      CkwsConfig& io_reorientedConfig)
    {
      io_reorientedConfig.dofValue (5, io_reorientedConfig.dofValue (5) 
				    + lateralAngle ());
      i_cfgValidator->validate (io_reorientedConfig);
      
      if (!io_reorientedConfig.isValid ())
	{
	  hppDout (warning,
		   "Reorient direct path end config orthogonally failed" 
		   << i_int);
	
	  // Keep original configuration.
	  io_reorientedConfig = i_originalConfig;
	  rotateDPEndConfig (i_path, i_int, io_reorientedConfig);

	  return KD_OK;
	}
      else 
	{
	  // Verify step direct path after direct path end configuration.
	  tryMakeOrthogonalDPForEndConfig (i_path, i_originalConfig,
					   i_nextDPEndConfig, i_dpValidator,
					   i_cfgValidator, i_int,
					   io_reorientedConfig);

	  return KD_OK;
	}
    }

    ktStatus Optimizer::
    tryMakeFrontalDPForEndConfig (const CkwsPathShPtr& i_path,
				  const CkwsConfig& i_originalConfig,
				  const CkwsConfig& i_nextDPEndConfig,
				  const CkwsValidatorDPCollisionShPtr&
				  i_dpValidator,
				  const CkwsValidatorCfgCollisionShPtr&
				  i_cfgValidator,
				  unsigned int i_int,
				  CkwsConfig& io_reorientedConfig)
    {
      CkwsConfig nextStepDPCfg (device ());
      
      // FIXME: Change third parameter to nextDPEndCfg if possible.
      if (KD_ERROR == nextStepConfig (i_originalConfig, io_reorientedConfig,
				      i_nextDPEndConfig, nextStepDPCfg, LATERAL,
				      i_cfgValidator, nextStepDPCfg))
	{
	  hppDout (warning, "nextStepConfig LATERAL is not valid "
		   << i_int);
		
	  // Keep original configuration and rotate it.
	  io_reorientedConfig = i_originalConfig;
	  rotateDPEndConfig (i_path, i_int, io_reorientedConfig);
	}
      else
	{
	  // Validate step direct path.
	  hppDout (notice, "nextStepConfig LATERAL is valid " 
		   << i_int);
	  
	  if (KD_ERROR == tryMakeStepDP (io_reorientedConfig, nextStepDPCfg,
					 i_dpValidator))
	    {
	      hppDout (error, "tryMakeStepDP is not valid " << i_int);
	      
	      tryOrthogonalDPEndConfig (i_path, i_originalConfig,
					i_nextDPEndConfig, i_dpValidator,
					i_cfgValidator, i_int,
					io_reorientedConfig);
	    }
	}
      
      return KD_OK;
    }

    ktStatus Optimizer::
    tryMakeOrthogonalDPForEndConfig (const CkwsPathShPtr& i_path,
				     const CkwsConfig& i_originalConfig,
				     const CkwsConfig& i_nextDPEndConfig,
				     const CkwsValidatorDPCollisionShPtr&
				     i_dpValidator,
				     const CkwsValidatorCfgCollisionShPtr&
				     i_cfgValidator,
				     unsigned int i_int,
				     CkwsConfig& io_reorientedConfig)
    {
      CkwsConfig nextStepDPCfg (device ());
      
      // FIXME: Change third parameter to nextDPEndCfg if possible.
      if (KD_ERROR == nextStepConfig (i_originalConfig, io_reorientedConfig,
				      i_nextDPEndConfig, nextStepDPCfg, LATERAL,
				      i_cfgValidator, nextStepDPCfg))
	{
	  hppDout (warning, "nextStepConfig LATERAL is not valid " << i_int);
		
	  // Keep original configuration and rotate it.
	  io_reorientedConfig = i_originalConfig;
	  rotateDPEndConfig (i_path, i_int, io_reorientedConfig);
	}
      else
	{
	  // Validate step direct path.
	  if (KD_ERROR == tryMakeStepDP (io_reorientedConfig, nextStepDPCfg,
					 i_dpValidator))
	    {
	      hppDout (error, "tryMakeStepDP is not valid " << i_int);
	      
	      // Keep original configuration and rotate it.
	      io_reorientedConfig = i_originalConfig;
	      rotateDPEndConfig (i_path, i_int, io_reorientedConfig);
	    }
	}

      return KD_OK;
    }

    ktStatus Optimizer::getOriginalConfig (const CkwsPathShPtr& i_path,
					   unsigned int i_dpIndexInt,
					   unsigned int i_stepIndexInt,
					   unsigned int i_stepNumberInt,
					   CkwsConfig& o_config)
    {
      CkwsDirectPathShPtr directPath 
	= CkwsDirectPath::createCopy (i_path->directPath (i_dpIndexInt));
      
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
					const CkwsConfig& i_nextDPEndConfig,
					unsigned int i_orientation,
					const CkwsValidatorCfgCollisionShPtr&
					i_cfgValidator,
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
      
      i_cfgValidator->validate (o_config);

      if (!o_config.isValid ())
	{
	  hppDout (error, "Intermediate configuration is not valid");
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
		   const CkwsConfig& i_endConfig,
		   const CkwsValidatorDPCollisionShPtr& i_dpValidator)
    {
      if (i_beginConfig.isEquivalent (i_endConfig))
	{
	  hppDout (error, "StepDP was not made.");
	  return KD_ERROR;
	}

      CkwsSMLinearShPtr linearSM = CkwsSMLinear::create ();
      CkwsDirectPathShPtr stepDP 
	= linearSM->makeDirectPath (i_beginConfig, i_endConfig);
      i_dpValidator->validate (*stepDP);
      
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
		  const CkwsConfig& i_nextDPEndConfig,
		  const CkwsValidatorDPCollisionShPtr&
		  i_dpValidator,
		  const CkwsValidatorCfgCollisionShPtr&
		  i_cfgValidator,
		  unsigned int i_int,
		  const unsigned int i_nbSteps,
		  CkwsConfig& io_lastConfig,
		  CkwsPathShPtr& io_path)
    {
      // Get next step configuration with frontal orientation.
      ktStatus success;
      CkwsConfig nextStepCfg (device ());
      if (i_int == i_nbSteps - 1)
	{
	  nextStepCfg = i_dpEndConfig;
	  success = KD_OK;
	}
      else
	{
	  success = nextStepConfig (i_originalConfig, io_lastConfig,
				    i_dpEndConfig, i_nextDPEndConfig, FRONTAL,
				    i_cfgValidator, nextStepCfg);
	}
      
      if (KD_ERROR == success)
	{
	  // Get next step configuratio with lateral orientation.
	  tryLateralStepConfig (i_originalConfig, i_dpEndConfig,
				i_nextDPEndConfig, i_dpValidator,
				i_cfgValidator, i_int, i_nbSteps, io_lastConfig,
				nextStepCfg, io_path);
	}
      else
	{
	  // Try to append step DP with frontal orientation.
	  tryAppendFrontalStepDP (i_originalConfig, i_dpEndConfig,
				  i_nextDPEndConfig, i_dpValidator,
				  i_cfgValidator, i_int, i_nbSteps,
				  io_lastConfig, nextStepCfg,
				  io_path);
	}

      return KD_OK;
    }

    ktStatus Optimizer::
    tryLateralStepConfig (const CkwsConfig& i_originalConfig,
			  const CkwsConfig& i_dpEndConfig,
			  const CkwsConfig i_nextDPEndConfig,
			  const CkwsValidatorDPCollisionShPtr&
			  i_dpValidator,
			  const CkwsValidatorCfgCollisionShPtr&
			  i_cfgValidator,
			  unsigned int i_int,
			  const unsigned int i_nbSteps,
			  CkwsConfig& io_lastConfig,
			  CkwsConfig& io_reorientedConfig,
			  CkwsPathShPtr& io_path)
    {
      if (KD_ERROR == nextStepConfig (i_originalConfig, io_lastConfig,
				      i_dpEndConfig, i_nextDPEndConfig,
				      LATERAL_ADJUSTED, i_cfgValidator,
				      io_reorientedConfig))
	{
	  // Get next step configuration with original configuration.
	  tryOriginalStepConfig (i_originalConfig, i_dpEndConfig,
				 i_nextDPEndConfig, i_dpValidator,
				 i_cfgValidator, i_int, i_nbSteps, io_lastConfig,
				 io_reorientedConfig, io_path);
	}
      else 
	{
	  // Try to append step DP with lateral orientation.
	  tryAppendLateralStepDP (i_originalConfig, i_dpEndConfig,
				  i_nextDPEndConfig, i_dpValidator,
				  i_cfgValidator, i_int, i_nbSteps,
				  io_lastConfig, io_reorientedConfig,
				  io_path);
	}
      
      return KD_OK;
    }

    ktStatus Optimizer::
    tryOriginalStepConfig (const CkwsConfig& i_originalConfig,
			   const CkwsConfig& i_dpEndConfig,
			   const CkwsConfig i_nextDPEndConfig,
			   const CkwsValidatorDPCollisionShPtr&
			   i_dpValidator,
			   const CkwsValidatorCfgCollisionShPtr&
			   i_cfgValidator,
			   unsigned int i_int,
			   const unsigned int i_nbSteps,
			   CkwsConfig& io_lastConfig,
			   CkwsConfig& io_reorientedConfig,
			   CkwsPathShPtr& io_path)
    {
      nextStepConfig (i_originalConfig, io_lastConfig, i_dpEndConfig,
		      i_nextDPEndConfig, ORIGINAL, i_cfgValidator,
		      io_reorientedConfig);
      
      // Try to append step DP with original orientation.
      tryAppendOriginalStepDP (i_originalConfig, i_dpEndConfig,
			       i_nextDPEndConfig, i_dpValidator, i_cfgValidator,
			       i_int, i_nbSteps, io_lastConfig,
			       io_reorientedConfig, io_path);
      
      return KD_OK;
    }   

    ktStatus Optimizer::
    tryAppendFrontalStepDP (const CkwsConfig& i_originalConfig,
			    const CkwsConfig& i_dpEndConfig,
			    const CkwsConfig i_nextDPEndConfig,
			    const CkwsValidatorDPCollisionShPtr&
			    i_dpValidator,
			    const CkwsValidatorCfgCollisionShPtr&
			    i_cfgValidator,
			    unsigned int i_int,
			    const unsigned int i_nbSteps,
			    CkwsConfig& io_lastConfig,
			    CkwsConfig& io_reorientedCfg,
			    CkwsPathShPtr& io_path)
    {
      if (io_lastConfig.isEquivalent (io_reorientedCfg))
	{
	  hppDout (error, "StepDP was not made.");
	  return KD_ERROR;
	}

      CkwsSMLinearShPtr linearSM = CkwsSMLinear::create ();
      CkwsDirectPathShPtr stepDP 
	= linearSM->makeDirectPath (io_lastConfig, io_reorientedCfg);
      i_dpValidator->validate (*stepDP);
      
      if (!stepDP->isValid ())
	{
	  // Get next step configuration with lateral orientation.
	  tryLateralStepConfig (i_originalConfig, i_dpEndConfig,
				i_nextDPEndConfig, i_dpValidator,
				i_cfgValidator, i_int, i_nbSteps, io_lastConfig,
				io_reorientedCfg, io_path);
	}
      else
	{
	  // Append step direct path with frontal orientation.
	  if (KD_ERROR == io_path->appendDirectPath (stepDP))
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
			    const CkwsConfig i_nextDPEndConfig,
			    const CkwsValidatorDPCollisionShPtr&
			    i_dpValidator,
			    const CkwsValidatorCfgCollisionShPtr&
			    i_cfgValidator,
			    unsigned int i_int,
			    const unsigned int i_nbSteps,
			    CkwsConfig& io_lastConfig,
			    CkwsConfig& io_reorientedCfg,
			    CkwsPathShPtr& io_path)
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
      
      CkwsSMLinearShPtr linearSM = CkwsSMLinear::create ();
      CkwsDirectPathShPtr stepDP 
	= linearSM->makeDirectPath (io_lastConfig, io_reorientedCfg);
      i_dpValidator->validate (*stepDP);
      
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
	  i_dpValidator->validate (*stepDP);

	  if (!stepDP->isValid ())
	    {
	      // Try to orient previous config lateral and make new direct path.
	      io_reorientedCfg.dofValue (5, io_reorientedCfg.dofValue (5)
					 + M_PI);
	      
	      if (KD_ERROR ==
		  tryPreviousLateralStepConfig (i_originalConfig, i_dpEndConfig,
						i_nextDPEndConfig, i_dpValidator,
						i_cfgValidator, i_int, i_nbSteps,
						io_lastConfig, io_reorientedCfg,
						io_path))
		{
		  hppDout (notice, "Trying original config.");
		  tryOriginalStepConfig (i_originalConfig, i_dpEndConfig,
					 i_nextDPEndConfig, i_dpValidator,
					 i_cfgValidator, i_int, i_nbSteps,
					 io_lastConfig, io_reorientedCfg,
					 io_path);
		}
	    }
	  else
	    {
	      // Append step direct path with lateral orientation.
	      if (KD_ERROR == io_path->appendDirectPath (stepDP))
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
	  if (KD_ERROR == io_path->appendDirectPath (stepDP))
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
				  const CkwsConfig i_nextDPEndConfig,
				  const CkwsValidatorDPCollisionShPtr&
				  i_dpValidator,
				  const CkwsValidatorCfgCollisionShPtr&
				  i_cfgValidator,
				  unsigned int i_int,
				  const unsigned int i_nbSteps,
				  CkwsConfig& io_lastConfig,
				  CkwsConfig& io_reorientedConfig,
				  CkwsPathShPtr& io_path)
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
	  io_path->extractToDirectPath (io_path->countDirectPaths () - 1);
	  io_path->getConfigAtEnd (io_lastConfig);
	}
	  
      // Replace it with one where last configuration is oriented
      // laterally.
      CkwsConfig lateralCfg (device ());
      if (KD_ERROR == nextStepConfig (i_originalConfig, io_lastConfig,
				      i_dpEndConfig, i_nextDPEndConfig,
				      LATERAL_ADJUSTED, i_cfgValidator,
				      lateralCfg))
	{
	  // FIXME: What to do in this case.
	  hppDout (error, "Previous lateral configuration not valid.");
	  return KD_OK;
	}
      else
	{
	  hppDout (notice, "Appending previous lateral step DP.");
	  tryAppendLateralStepDP (i_originalConfig, i_dpEndConfig,
				  i_nextDPEndConfig, i_dpValidator,
				  i_cfgValidator, i_int, i_nbSteps,io_lastConfig,
				  lateralCfg, io_path);
	  
	  hppDout (notice, "Appending lateral step DP.");
	  tryAppendLateralStepDP (i_originalConfig, i_dpEndConfig,
				  i_nextDPEndConfig, i_dpValidator,
				  i_cfgValidator, i_int, i_nbSteps, io_lastConfig,
				  io_reorientedConfig, io_path);
	}
	
      return KD_OK;
    }

    ktStatus Optimizer::
    tryAppendOriginalStepDP (const CkwsConfig& i_originalConfig,
			     const CkwsConfig& i_dpEndConfig,
			     const CkwsConfig i_nextDPEndConfig,
			     const CkwsValidatorDPCollisionShPtr&
			     i_dpValidator,
			     const CkwsValidatorCfgCollisionShPtr&
			     i_cfgValidator,
			     unsigned int i_int,
			     const unsigned int i_nbSteps,
			     CkwsConfig& io_lastConfig,
			     CkwsConfig& io_reorientedCfg,
			     CkwsPathShPtr& io_path)
    {
      if (io_lastConfig.isEquivalent (io_reorientedCfg))
	{
	  hppDout (error, "StepDP was not made.");
	  return KD_ERROR;
	}

      CkwsSMLinearShPtr linearSM = CkwsSMLinear::create ();
      CkwsDirectPathShPtr stepDP 
	= linearSM->makeDirectPath (io_lastConfig, io_reorientedCfg);
      i_dpValidator->validate (*stepDP);
      
      if (!stepDP->isValid ())
	{
	  // FIXME: find a solution for this case. Maybe bakcup to
	  // previous config and use original orientation as well.
	  hppDout (error, "Appending original step direct path with collision "
		   << i_int);
	  if (KD_ERROR == io_path->appendDirectPath (stepDP))
	    {
	      hppDout (error, "Could not append original step direct path " 
		       << i_int);
	      return KD_ERROR;
	    }
	  io_lastConfig = io_reorientedCfg;
	}
      else
	{
	  // Append step direct path with original orientation.
	  if (KD_ERROR == io_path->appendDirectPath (stepDP))
	    {
	      hppDout (error, "Could not append original step direct path " 
		       << i_int);
	      return KD_ERROR;
	    }
	  io_lastConfig = io_reorientedCfg;
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
