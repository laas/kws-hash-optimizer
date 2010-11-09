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

//FIXME {update later hpp-util in robotpkg to include sstream}
#include <sstream>
#include <hpp/util/debug.hh>

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
	  return KD_ERROR;
	}
      
      *io_path = *outPath;

      return KD_OK;
    }   

    ktStatus Optimizer::alignPathConfigs (const CkwsPathShPtr& i_path,
					       CkwsPathShPtr& o_path)
    {
      CkwsValidatorDPCollisionShPtr dpValidator;
      CkwsValidatorCfgCollisionShPtr cfgValidator;
      if (KD_ERROR == retrieveValidators (i_path, dpValidator, cfgValidator))
	return KD_ERROR;
    
      unsigned int configsNumber = i_path->countConfigurations ();
      CkwsPathShPtr outPath = CkwsPath::create (device ());
      
      // if (configsNumber ==2)
      // 	{
      // 	  *o_path = *i_path;
      // 	  return KD_OK;
      // 	}

      for (unsigned int i = 0; i < configsNumber - 1; i++)
	{
	  // std::cout << "alignPathConfigs: " << i << std::endl;
	  appendModifiedDP (i_path, dpValidator, cfgValidator, i,
			    outPath);
	
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

    ktStatus Optimizer::alignEndConfig
    (const CkwsPathShPtr& i_path,
     const CkwsValidatorDPCollisionShPtr& i_dpValidator,
     const CkwsValidatorCfgCollisionShPtr& i_cfgValidator,
     unsigned int i_int,
     CkwsConfig& o_config)
    {
      unsigned int nbConfig = i_path->countConfigurations ();
      if (i_int == nbConfig - 2)
	{
	  i_path->getConfigAtEnd (o_config);
	  return KD_OK;
	}
    
      CkwsConfig ithStartCfg (device ());
      CkwsConfig ithEndCfg (device ());
      CkwsConfig ithNextEndCfg (device ());
      i_path->getConfiguration (i_int, ithStartCfg);
      i_path->getConfiguration (i_int + 1, ithEndCfg);
      i_path->getConfiguration (i_int + 2, ithNextEndCfg);

      CkwsDirectPathShPtr ithDP
	= CkwsDirectPath::createCopy (i_path->directPath (i_int));
      CkwsDirectPathShPtr ithNextDP 
	= CkwsDirectPath::createCopy (i_path->directPath (i_int + 1));
      double stepSize = human_size_/6;
      unsigned int ithNbSteps = (int)(ithDP->length () / stepSize);
      unsigned int ithNextNbSteps = (int)(ithNextDP->length () / stepSize);
    
      if ((ithNbSteps < minStepsNb ()) || (ithNextNbSteps < minStepsNb ()))
	{
	  o_config = ithEndCfg;
	  return KD_OK;
	}

      double ithDeltaX = ithEndCfg.dofValue (0) - ithStartCfg.dofValue (0);
      double ithDeltaY = ithEndCfg.dofValue (1) - ithStartCfg.dofValue (1);
      double ithNextDeltaX = ithNextEndCfg.dofValue (0) - ithEndCfg.dofValue (0);
      double ithNextDeltaY = ithNextEndCfg.dofValue (1) - ithEndCfg.dofValue (1);
      CkwsConfig interCfg (device ());
      CkwsConfig endCfg (device ());
      endCfg = ithEndCfg;
      endCfg.dofValue (5, (atan2 (ithDeltaY, ithDeltaX) 
			   + atan2 (ithNextDeltaY, ithNextDeltaX)) / 2);
      i_cfgValidator->validate (endCfg);
    
      ktStatus success = KD_OK;;
      if (!endCfg.isValid ())
	{
	  // std::cout << "alignEndConfig: aligned config invalid" << std::endl;
	  endCfg.dofValue (5, (atan2 (ithDeltaY, ithDeltaX) 
			       + atan2 (ithNextDeltaY, ithNextDeltaX)) / 2 + M_PI / 2);
	  i_cfgValidator->validate (endCfg);
	  if (!endCfg.isValid ())
	    {
	      // std::cout << "alignEndConfig: orthogonal config invalid" << std::endl;
	      success = KD_ERROR;
	    }
	  else 
	    {
	      if (KD_ERROR == intermediateConfig (ithEndCfg, ithNextEndCfg, true,
						  false, true, device (),
						  i_cfgValidator,	interCfg))
		{
		  // std::cout << "alignEndConfig: inter orth Cfg invalid"  << std::endl;
		  o_config = ithEndCfg;
		}
	      else 
		{
		  CkwsSMLinearShPtr linearSM = CkwsSMLinear::create ();
		  CkwsDirectPathShPtr stepDP 
		    = linearSM->makeDirectPath (endCfg, interCfg);
		  i_dpValidator->validate (*stepDP);
		  if (!stepDP->isValid ())
		    {
		      // std::cout << "alignEndConfig: orth step DP invalid" << std::endl;
		      success = KD_ERROR;
		    }
		  else 
		    {
		      // std::cout << "alignEndConfig: aligned orth Config valid" 
		      // 	      << std::endl;
		      o_config = endCfg;
		    }
		}
	    }
	  if (success != KD_OK)
	    {
	      // std::cout << "trying second orthogonal" << std::endl;
	      endCfg.dofValue (5, (atan2 (ithDeltaY, ithDeltaX) 
				   + atan2 (ithNextDeltaY, ithNextDeltaX)) / 2 
			       - M_PI / 2);
	      i_cfgValidator->validate (endCfg);
	      if (!endCfg.isValid ())
		{
		  // std::cout << "alignEndConfig: orthogonal config invalid" 
		  // 	  << std::endl;
		  o_config = ithEndCfg;
		}
	      else 
		{
		  CkwsSMLinearShPtr linearSM = CkwsSMLinear::create ();
		  CkwsDirectPathShPtr stepDP 
		    = linearSM->makeDirectPath (endCfg, interCfg);
		  i_dpValidator->validate (*stepDP);
		  if (!stepDP->isValid ())
		    {
		      // std::cout << "alignEndConfig: orth step DP invalid" 
		      // 	      << std::endl;
		      o_config = ithEndCfg;
		    }
		  else 
		    {
		      // std::cout << "alignEndConfig: aligned orth Config valid" 
		      // 	      << std::endl;
		      o_config = endCfg;
		    }
		}
	    }
	}
      else
	{
	  if (KD_ERROR == intermediateConfig (ithEndCfg, ithNextEndCfg, true, false,
					      true, device (), i_cfgValidator,
					      interCfg))
	    {
	      // std::cout << "alignEndConfig: inter Cfg invalid"  << std::endl;
	      o_config = ithEndCfg;
	    }
	  else 
	    {
	      CkwsSMLinearShPtr linearSM = CkwsSMLinear::create ();
	      CkwsDirectPathShPtr stepDP 
		= linearSM->makeDirectPath (endCfg, interCfg);
	      i_dpValidator->validate (*stepDP);
	      if (!stepDP->isValid ())
		{
		  // std::cout << "alignEndConfig: step DP invalid" << std::endl;
		
		  endCfg.dofValue (5, (atan2 (ithDeltaY, ithDeltaX) 
				       + atan2 (ithNextDeltaY, ithNextDeltaX)) / 2 
				   + M_PI / 2);
		  i_cfgValidator->validate (endCfg);
		  if (!endCfg.isValid ())
		    {
		      // std::cout << "alignEndConfig: orthogonal config invalid" 
		      // 	      << std::endl;
		      success = KD_ERROR;
		    }
		  else 
		    {
		      CkwsDirectPathShPtr stepDP 
			= linearSM->makeDirectPath (endCfg, interCfg);
		      i_dpValidator->validate (*stepDP);
		      if (!stepDP->isValid ())
			{
			  // std::cout << "alignEndConfig: orth step DP invalid" 
			  // 	  << std::endl;
			  success = KD_ERROR;
			}
		      else 
			{
			  // std::cout << "alignEndConfig: aligned orth Config valid" 
			  // 	  << std::endl;
			  o_config = endCfg;
			}
		    }
		  if (success != KD_OK)
		    {
		      // std::cout << "trying other orthogonal" << std::endl;
		      endCfg.dofValue (5, (atan2 (ithDeltaY, ithDeltaX) 
					   + atan2 (ithNextDeltaY, ithNextDeltaX)) / 2 
				       - M_PI / 2);
		      i_cfgValidator->validate (endCfg);
		      if (!endCfg.isValid ())
			{
			  // std::cout << "alignEndConfig: orthogonal config invalid" 
			  // 	  << std::endl;
			  o_config = ithEndCfg;
			}
		      else 
			{
			  CkwsDirectPathShPtr stepDP 
			    = linearSM->makeDirectPath (endCfg, interCfg);
			  i_dpValidator->validate (*stepDP);
			  if (!stepDP->isValid ())
			    {
			      // std::cout << "alignEndConfig: orth step DP invalid" 
			      // 	      << std::endl;
			      o_config = ithEndCfg;
			    }
			  else 
			    {
			      // std::cout
			      //   << "alignEndConfig: aligned orth Config valid" 
			      //   << std::endl;
			      o_config = endCfg;
			    }
			}
		    }
		}
	      else 
		{
		  // std::cout << "alignEndConfig: aligned Config valid" << std::endl;
		  o_config = endCfg;
		}
	    }
	}
      return KD_OK;
    }

    ktStatus Optimizer::
    intermediateConfig (const CkwsConfig& i_leftConfig,
			const CkwsConfig& i_rightConfig,
			bool i_start,
			bool i_end,
			bool i_lateral,
			const CkwsDeviceShPtr& i_device,
			const CkwsValidatorCfgCollisionShPtr& i_cfgValidator,
			CkwsConfig& o_config)
    {
      bool splitInHalf = false;
      double deltaX = i_rightConfig.dofValue (0) - i_leftConfig.dofValue (0);
      double deltaY = i_rightConfig.dofValue (1) - i_leftConfig.dofValue (1);
      if (true)
	{
	  double vectorNorm = sqrt (pow (deltaX, 2) + pow (deltaY, 2));
	  if (human_size_/6 < vectorNorm/2)
	    {
	      if (i_start)
		{
		  o_config.dofValue (0, i_leftConfig.dofValue (0) 
				     + human_size_/6 * deltaX/vectorNorm);
		  o_config.dofValue (1, i_leftConfig.dofValue (1) 
				     + human_size_/6 * deltaY/vectorNorm);
		}
	      if (i_end)
		{
		  o_config.dofValue (0, i_rightConfig.dofValue (0) 
				     - human_size_/6 * deltaX/vectorNorm);
		  o_config.dofValue (1, i_rightConfig.dofValue (1) 
				     - human_size_/6 * deltaY/vectorNorm);
		}
	      if (!i_start && !i_end)
		splitInHalf = true; 
	    }
	  else splitInHalf = true;
	}
      else splitInHalf = true;
      if (splitInHalf)
	{
	  o_config.dofValue 
	    (0, (i_leftConfig.dofValue (0) + i_rightConfig.dofValue (0))/2);
	  o_config.dofValue 
	    (1, (i_leftConfig.dofValue (1) + i_rightConfig.dofValue (1))/2);
	}
      if (i_lateral)
	o_config.dofValue (5, atan2 (deltaY, deltaX) + M_PI/2);
      else o_config.dofValue (5, atan2 (deltaY, deltaX));
      i_cfgValidator->validate (o_config);
      if (!o_config.isValid ())
	return KD_ERROR;
      else return KD_OK;
    }
 
    ktStatus Optimizer::
    appendModifiedDP (const CkwsPathShPtr& i_path,
		      const CkwsValidatorDPCollisionShPtr& i_dpValidator,
		      const CkwsValidatorCfgCollisionShPtr& i_cfgValidator,
		      unsigned int i_int,
		      CkwsPathShPtr& io_path)
    {
      CkwsDirectPathShPtr i_dp 
	= CkwsDirectPath::createCopy (i_path->directPath (i_int));
      CkwsSMLinearShPtr linearSM = CkwsSMLinear::create ();
      double stepSize = human_size_/6;
    
      CkwsConfig startCfg (device ());
      if (i_int == 0)
	startCfg = i_dp->startConfiguration ();
      else io_path->getConfigAtEnd (startCfg);
      CkwsConfig endCfg (device ());
      alignEndConfig (i_path, i_dpValidator, i_cfgValidator, i_int, endCfg); 
    
      unsigned int nbSteps = (int)(i_dp->length () / stepSize); 
      if (nbSteps < minStepsNb ())
	{
	  CkwsDirectPathShPtr directPath 
	    = linearSM->makeDirectPath (startCfg, endCfg);
	  // validate direct Path
	  if (KD_ERROR == io_path->appendDirectPath (directPath))
	    {
	      std::cout << "*** appendModifiedPath: could not append DP" << std::endl;
	      return KD_ERROR;
	    }
	  else 
	    {
	      // std::cout << "*** appendModifiedPath: appended DP" << std::endl;
	      return KD_OK;
	    }
	}
     
      CkwsConfig currentModCfg (i_dp->device ());
      CkwsConfig previousCfg = startCfg; 
      CkwsConfig lastCfg = startCfg;
      ktStatus success;
      enum {STRAIGHT, LATERAL};
      unsigned int mode = LATERAL;
      unsigned int i = 0;
      while (i < nbSteps)
	{
	  // std::cout << "*** makeModifiedPath: " << i << std::endl;
	  if (i == nbSteps - 1)
	    {
	      currentModCfg = endCfg;
	      success = KD_OK;
	    }
	  else
	    {
	      success = intermediateConfig (previousCfg, endCfg, true, false,
					    false, i_dp->device (),
					    i_cfgValidator, currentModCfg);   
	    }

	  if (KD_ERROR == success)
	    {
	      // std::cout << "*** config invalid" << std::endl;
	      switch (mode)
		{
		case (STRAIGHT):
		  {
		    // std::cout << "*** STRAIGHT mode" << std::endl;
		    if (KD_ERROR == intermediateConfig (previousCfg, endCfg, true,
							false, true, i_dp->device (),
							i_cfgValidator, currentModCfg))
		      unsigned int antonio = 0;
		    // std::cout << "*** lateral Cfg invalid!!" << std::endl;
		    CkwsDirectPathShPtr stepDP 
		      = linearSM->makeDirectPath (previousCfg, currentModCfg); 
		    i_dpValidator->validate (*stepDP);
		    if (!stepDP->isValid ())
		      {
			// std::cout << "*** step DP invalid" << std::endl;
			CkwsConfig backUpCfg (i_dp->device ());
			backUpConfig (lastCfg, previousCfg, device (),
				      i_dpValidator, i_cfgValidator, backUpCfg);
			CkwsConfig transitionCfg (i_dp->device ());
			intermediateConfig (backUpCfg, endCfg, true, false, true,
					    device (), i_cfgValidator, transitionCfg);
			CkwsDirectPathShPtr sectionDP
			  = linearSM->makeDirectPath (lastCfg, backUpCfg);
			if (KD_ERROR == io_path->appendDirectPath (sectionDP))
			  {
			    std::cout << "*** could not append section DP" 
				      << std::endl;
			    return KD_ERROR;
			  }
			// else std::cout << "*** section DP appended" << std::endl;
			CkwsDirectPathShPtr stepDP 
			  = linearSM->makeDirectPath (backUpCfg, transitionCfg);
			if (KD_ERROR == io_path->appendDirectPath (stepDP))
			  {
			    std::cout << "*** could not append step DP" << std::endl;
			    return KD_ERROR;
			  }
			// else std::cout << "*** step DP appended" << std::endl;
			currentModCfg = transitionCfg;
			lastCfg = transitionCfg;
		      }
		    else
		      {
			if (lastCfg != previousCfg)
			  {
			    CkwsDirectPathShPtr sectionDP 
			      = linearSM->makeDirectPath (lastCfg, previousCfg);
			    if (KD_ERROR == io_path->appendDirectPath (sectionDP))
			      {
				std::cout << "*** could not append section DP" 
					  << std::endl;
				return KD_ERROR;
			      }
			    // else std::cout << "*** section DP appended" << std::endl;
			  }
			if (KD_ERROR == io_path->appendDirectPath (stepDP))
			  {
			    std::cout << "*** could not append step DP" << std::endl;
			    return KD_ERROR;
			  }
			// else std::cout << "*** step DP appended" << std::endl;
			lastCfg = currentModCfg;
		      }
		    mode = LATERAL;
		    break;
		  }
		case (LATERAL):
		  {
		    // std::cout << "*** LATERAL mode" << std::endl;
		    if (KD_ERROR == intermediateConfig (previousCfg, endCfg, true,
							false, true, i_dp->device (),
							i_cfgValidator, currentModCfg))
		      // std::cout << "*** lateral Cfg invalid!!" << std::endl;
		      if (i == 0)
			{
			  CkwsDirectPathShPtr stepDP 
			    = linearSM->makeDirectPath (previousCfg, currentModCfg);
			  if (KD_ERROR == io_path->appendDirectPath (stepDP))
			    {
			      std::cout << "*** could not append step DP" << std::endl;
			      return KD_ERROR;
			    }
			  // else std::cout << "*** step DP appended" << std::endl;
			  lastCfg = currentModCfg;
			}
		    break;
		  }
		}
	    }

	  else 
	    {
	      CkwsDirectPathShPtr stepDP 
		= linearSM->makeDirectPath (previousCfg, currentModCfg); 
	      i_dpValidator->validate (*stepDP);
	      if (!stepDP->isValid ())
		{
		  // std::cout << "*** step DP invalid" << std::endl;
		  switch (mode)
		    {
		    case (STRAIGHT):
		      {
			// std::cout << "*** STRAIGHT mode" << std::endl;
			if (i == nbSteps - 1)
			  {
			    currentModCfg = endCfg;
			    success = KD_OK;
			  }
			else
			  {
			    success 
			      = intermediateConfig (previousCfg, endCfg, true, false, 
						    true, i_dp->device (),
						    i_cfgValidator, currentModCfg);   
			  }
			if (KD_ERROR == success)
			  unsigned antonio = 0;
			// std::cout << "*** lateral Cfg invalid!!" << std::endl;
			CkwsDirectPathShPtr stepDP 
			  = linearSM->makeDirectPath (previousCfg, currentModCfg); 
			i_dpValidator->validate (*stepDP);
			if (!stepDP->isValid ())
			  {
			    // std::cout << "*** step DP invalid" << std::endl;
			    CkwsConfig backUpCfg (i_dp->device ());
			    backUpConfig (lastCfg, previousCfg, i_dp->device (),
					  i_dpValidator, i_cfgValidator, backUpCfg);
			    CkwsConfig transitionCfg (i_dp->device ());
			    intermediateConfig (backUpCfg, endCfg, true, false, true,
						i_dp->device (), i_cfgValidator, 
						transitionCfg);
			    CkwsDirectPathShPtr sectionDP
			      = linearSM->makeDirectPath (lastCfg, backUpCfg);
			    if (KD_ERROR == io_path->appendDirectPath (sectionDP))
			      {
				std::cout << "*** could not append section DP" 
					  << std::endl;
				return KD_ERROR;
			      }
			    // else std::cout << "*** section DP appended" << std::endl;
			    CkwsDirectPathShPtr stepDP 
			      = linearSM->makeDirectPath (backUpCfg, transitionCfg);
			    if (KD_ERROR == io_path->appendDirectPath (stepDP))
			      {
				std::cout << "*** could not append step DP" 
					  << std::endl;
				return KD_ERROR;
			      }
			    // else std::cout << "*** step DP appended" << std::endl;
			    lastCfg = transitionCfg;
			    if (i == nbSteps - 1)
			      {
				CkwsDirectPathShPtr stepDP 
				  = linearSM->makeDirectPath (transitionCfg, endCfg);
				if (KD_ERROR == io_path->appendDirectPath (stepDP))
				  {
				    std::cout << "*** could not append step DP" 
					      << std::endl;
				    return KD_ERROR;
				  }
				// else std::cout << "*** step DP appended" << std::endl;
				lastCfg = endCfg;
			      }
			  }
			else
			  {
			    if (lastCfg != currentModCfg)
			      {
				CkwsDirectPathShPtr sectionDP 
				  = linearSM->makeDirectPath (lastCfg, currentModCfg);
				if (KD_ERROR == io_path->appendDirectPath (sectionDP))
				  {
				    std::cout << "*** could not append section DP" 
					      << std::endl;
				    return KD_ERROR;
				  }
				// else std::cout << "*** section DP appended" 
				// 		     << std::endl;
				lastCfg = currentModCfg;
			      }
			    if (i == nbSteps -1)
			      {
				if (lastCfg != previousCfg)
				  {
				    CkwsDirectPathShPtr sectionDP 
				      = linearSM->makeDirectPath (lastCfg,
								  previousCfg);
				    if (KD_ERROR
					== io_path->appendDirectPath (sectionDP))
				      {
					std::cout << "*** could not append section DP" 
						  << std::endl;
					return KD_ERROR;
				      }
				    // else std::cout << "*** section DP appended" 
				    // 		 << std::endl;
				  }
				CkwsDirectPathShPtr stepDP 
				  = linearSM->makeDirectPath (previousCfg, endCfg);
				if (KD_ERROR == io_path->appendDirectPath (stepDP))
				  {
				    std::cout << "*** could not append step DP" 
					      << std::endl;
				    return KD_ERROR;
				  }
				// else std::cout << "*** step DP appended" << std::endl;
				lastCfg = endCfg; 
			      }
			  }
			mode = LATERAL;
			break;
		      }
		    case (LATERAL):
		      {
			// std::cout << "*** LATERAL mode" << std::endl;
			if (lastCfg != previousCfg)
			  {
			    CkwsDirectPathShPtr sectionDP 
			      = linearSM->makeDirectPath (lastCfg, previousCfg);
			    if (KD_ERROR == io_path->appendDirectPath (sectionDP))
			      {
				std::cout << "*** could not append section DP" 
					  << std::endl;
				return KD_ERROR;
			      }
			    // else std::cout << "*** section DP appended" << std::endl;
			  }
			if (i == nbSteps - 1)
			  {
			    CkwsDirectPathShPtr stepDP 
			      = linearSM->makeDirectPath (previousCfg, endCfg);
			    if (KD_ERROR == io_path->appendDirectPath (stepDP))
			      {
				std::cout << "*** could not append step DP" 
					  << std::endl;
				return KD_ERROR;
			      }
			    // else std::cout << "*** step DP appended" << std::endl;
			    lastCfg = endCfg;
			  }
			else 
			  {
			    if (KD_ERROR 
				== intermediateConfig (previousCfg, endCfg, true,
						       false, true, i_dp->device (),
						       i_cfgValidator, currentModCfg))
			      unsigned int antonio = 0;
			    // std::cout << "*** lateral Cfg invalid!!" << std::endl;
			    CkwsDirectPathShPtr stepDP 
			      = linearSM->makeDirectPath (previousCfg, currentModCfg);
			    i_dpValidator->validate (*stepDP);
			    if (!stepDP->isValid ())
			      {
				/////
			      }
			    else
			      {
				if (KD_ERROR == io_path->appendDirectPath (stepDP))
				  {
				    std::cout << "*** could not append step DP" 
					      << std::endl;
				    return KD_ERROR;
				  }
				// else std::cout << "*** step DP appended" << std::endl;
				lastCfg = currentModCfg;
			      }
			  }
			break;
		      }
		    }
		}
	      else
		{
		  switch (mode)
		    {
		    case (STRAIGHT):
		      {
			// std::cout << "*** STRAIGHT mode" << std::endl;
			if (i == nbSteps - 1)
			  {
			    if (lastCfg != previousCfg)
			      {
				CkwsDirectPathShPtr sectionDP 
				  = linearSM->makeDirectPath (lastCfg, previousCfg);
				if (KD_ERROR == io_path->appendDirectPath (sectionDP))
				  {
				    std::cout << "*** could not append section DP" 
					      << std::endl;
				    return KD_ERROR;
				  }
				// else std::cout << "*** section DP appended" 
				// 		     << std::endl;
			      }
			    if (KD_ERROR == io_path->appendDirectPath (stepDP))
			      {
				std::cout << "*** could not append step DP" 
					  << std::endl;
				return KD_ERROR;
			      }
			  }
			break;
		      }
		    case (LATERAL):
		      {
			// std::cout << "*** LATERAL mode" << std::endl;
			if (lastCfg != previousCfg)
			  {
			    CkwsDirectPathShPtr sectionDP 
			      = linearSM->makeDirectPath (lastCfg, previousCfg);
			    if (KD_ERROR == io_path->appendDirectPath (sectionDP))
			      {
				std::cout << "*** could not append section DP" 
					  << std::endl;
				return KD_ERROR;
			      }
			    // else std::cout << "*** section DP appended" << std::endl;
			  }
			//////////validate stepDP!!!!!!
			if (KD_ERROR == io_path->appendDirectPath (stepDP))
			  {
			    std::cout << "*** could not append step DP" << std::endl;
			    return KD_ERROR;
			  }
			// else std::cout << "*** step DP appended" << std::endl;
			lastCfg = currentModCfg;
			mode = STRAIGHT;
			break;
		      }
		    }
		}
	    }
	  previousCfg = currentModCfg;
	  i++;
	}
      return KD_OK;
    }

    ktStatus 
    Optimizer::backUpConfig (const CkwsConfig& i_lastCfg,
				  const CkwsConfig& i_currentCfg,
				  const CkwsDeviceShPtr& i_device, 
				  const CkwsValidatorDPCollisionShPtr& i_dpValidator,
				  const CkwsValidatorCfgCollisionShPtr& i_cfgValidator,
				  CkwsConfig& o_config)
    {
      CkwsSMLinearShPtr linearSM = CkwsSMLinear::create ();
      CkwsConfig backUpCfg (i_device);
      CkwsConfig currentCfg  = i_currentCfg;
      double deltaX = currentCfg.dofValue (0) - i_lastCfg.dofValue (0);
      double deltaY = currentCfg.dofValue (1) - i_lastCfg.dofValue (1);
      while (currentCfg != i_lastCfg)
	{
	  currentCfg.dofValue (5, atan2 (deltaY, deltaX) + M_PI/2);
	  intermediateConfig (i_lastCfg, currentCfg, false, true, false, i_device,
			      i_cfgValidator, backUpCfg);
	  CkwsDirectPathShPtr stepDP 
	    = linearSM->makeDirectPath (backUpCfg, currentCfg);
	  i_dpValidator->validate (*stepDP);
	  if (!stepDP->isValid ())
	    {
	      // std::cout << "****** backUpConfig: stepDP invalid" << std::endl;
	      currentCfg = backUpCfg;
	    }
	  else 
	    {
	      o_config = backUpCfg;
	      return KD_OK;
	    }
	}
      // std::cout << "****** backUpConfig: could not find backUp config" << std::endl;
      return KD_ERROR;
    }

    ktStatus Optimizer::
    appendHashedDP (const CkwsPathShPtr& i_path,
		    const CkwsValidatorDPCollisionShPtr& i_dpValidator,
		    const CkwsValidatorCfgCollisionShPtr& i_cfgValidator,
		    unsigned int i_int,
		    CkwsPathShPtr& io_path){return KD_OK;}

    // FIXME {doxygen}
    ktStatus Optimizer::
    rotateDPEndConfig (const CkwsPathShPtr& i_path,
		       unsigned int i_int,
		       CkwsConfig& io_config){return KD_OK;}

    // FIXME {doxygen}
    ktStatus Optimizer::
    reorientDPEndConfig (const CkwsPathShPtr& i_path,
			 const CkwsValidatorDPCollisionShPtr&
			 i_dpValidator,
			 const CkwsValidatorCfgCollisionShPtr&
			 i_cfgValidator,
			 unsigned int i_int,
			 CkwsConfig& o_config){return KD_OK;}

    // FIXME {doxygen}
    ktStatus Optimizer::
    tryOrthogonalDPEndConfig (const CkwsPathShPtr& i_path,
			      const CkwsConfig& i_originalConfig,
			      const CkwsValidatorDPCollisionShPtr&
			      i_dpValidator,
			      const CkwsValidatorCfgCollisionShPtr&
			      i_cfgValidator,
			      unsigned int i_int,
			      CkwsConfig& io_reorientedConfig){return KD_OK;}

    // FIXME {doxygen}
    ktStatus Optimizer::
    tryMakeFrontalDPForEndConfig (const CkwsPathShPtr& i_path,
				  const CkwsConfig& i_originalConfig,
				  const CkwsValidatorDPCollisionShPtr&
				  i_dpValidator,
				  const CkwsValidatorCfgCollisionShPtr&
				  i_cfgValidator,
				  unsigned int i_int,
				  CkwsConfig& io_reorientedConfig){return KD_OK;}

    // FIXME {doxygen}
    ktStatus Optimizer::
    tryMakeOrthogonalDPForEndConfig (const CkwsPathShPtr& i_path,
				     const CkwsConfig& i_originalConfig,
				     const CkwsValidatorDPCollisionShPtr&
				     i_dpValidator,
				     const CkwsValidatorCfgCollisionShPtr&
				     i_cfgValidator,
				     unsigned int i_int,
				     CkwsConfig& io_reorientedConfig){return KD_OK;}

    // FIXME {doxygen}
    ktStatus Optimizer:: nextStepConfig (const CkwsConfig& i_beginConfig,
				     const CkwsConfig& i_endConfig,
				     const CkwsConfig& i_nextDPEndConfig,
				     unsigned int i_orientation,
				     const CkwsValidatorCfgCollisionShPtr&
				     i_cfgValidator,
				     CkwsConfig& o_config){return KD_OK;}
      
    // FIXME {doxygen}
    ktStatus Optimizer::
    adjustLateralConfig (const CkwsConfig& i_endConfig,
			 const CkwsConfig& i_nextDPEndConfig,
			 CkwsConfig& io_config){return KD_OK;}

    // FIXME {doxygen}
    ktStatus Optimizer::
    adjustOriginalConfig (const CkwsConfig& i_endConfig,
			  CkwsConfig& io_config){return KD_OK;}

    // FIXME {doxygen}
    ktStatus Optimizer::
    tryMakeStepDP (const CkwsConfig& i_beginConfig,
		   const CkwsConfig& i_endConfig,
		   const CkwsValidatorDPCollisionShPtr& i_dpValidator){return KD_OK;}
	
    // FIXME {doxygen}
    ktStatus Optimizer::
    appendStepDP (const CkwsConfig& i_dpEndConfig,
		  const CkwsConfig& i_nextDPEndConfig,
		  const CkwsValidatorDPCollisionShPtr&
		  i_dpValidator,
		  const CkwsValidatorCfgCollisionShPtr&
		  i_cfgValidator,
		  unsigned int i_int,
		  const unsigned int i_nbSteps,
		  CkwsConfig& io_lastConfig,
		  CkwsPathShPtr& io_path){return KD_OK;}

    // FIXME: doxygen
    ktStatus Optimizer::
    tryLateralStepConfig (const CkwsConfig& i_dpEndConfig,
			  const CkwsConfig i_nextDPEndConfig,
			  const CkwsValidatorDPCollisionShPtr&
			  i_dpValidator,
			  const CkwsValidatorCfgCollisionShPtr&
			  i_cfgValidator,
			  unsigned int i_int,
			  const unsigned int i_nbSteps,
			  CkwsConfig& io_lastConfig,
			  CkwsConfig& io_reorientedConfig,
			  CkwsPathShPtr& io_path){return KD_OK;}

    // FIXME: doxygen      
    ktStatus Optimizer::
    tryOriginalStepConfig (const CkwsConfig& i_dpEndConfig,
			   const CkwsConfig i_nextDPEndConfig,
			   const CkwsValidatorDPCollisionShPtr&
			   i_dpValidator,
			   const CkwsValidatorCfgCollisionShPtr&
			   i_cfgValidator,
			   unsigned int i_int,
			   const unsigned int i_nbSteps,
			   CkwsConfig& io_lastConfig,
			   CkwsConfig& io_reorientedConfig,
			   CkwsPathShPtr& io_path){return KD_OK;}

    // FIXME: doxygen
    ktStatus Optimizer::
    tryAppendFrontalStepDP (const CkwsConfig& i_dpEndConfig,
			    const CkwsConfig i_nextDPEndConfig,
			    const CkwsValidatorDPCollisionShPtr&
			    i_dpValidator,
			    const CkwsValidatorCfgCollisionShPtr&
			    i_cfgValidator,
			    unsigned int i_int,
			    const unsigned int i_nbSteps,
			    CkwsConfig& io_lastConfig,
			    CkwsConfig& io_reorientedCfg,
			    CkwsPathShPtr& io_path){return KD_OK;}

    // FIXME: doxygen
    ktStatus Optimizer::
    tryAppendLateralStepDP (const CkwsConfig& i_dpEndConfig,
			    const CkwsConfig i_nextDPEndConfig,
			    const CkwsValidatorDPCollisionShPtr&
			    i_dpValidator,
			    const CkwsValidatorCfgCollisionShPtr&
			    i_cfgValidator,
			    unsigned int i_int,
			    const unsigned int i_nbSteps,
			    CkwsConfig& io_lastConfig,
			    CkwsConfig& io_reorientedCfg,
			    CkwsPathShPtr& io_path){return KD_OK;}

    // FIXME: doxygen
    ktStatus Optimizer::
    tryAppendOriginalStepDP (const CkwsConfig& i_dpEndConfig,
			     const CkwsConfig i_nextDPEndConfig,
			     const CkwsValidatorDPCollisionShPtr&
			     i_dpValidator,
			     const CkwsValidatorCfgCollisionShPtr&
			     i_cfgValidator,
			     unsigned int i_int,
			     const unsigned int i_nbSteps,
			     CkwsConfig& io_lastConfig,
			     CkwsConfig& io_reorientedCfg,
			     CkwsPathShPtr& io_path){return KD_OK;}
     
    Optimizer::Optimizer (unsigned int i_nbLoops,
			  double i_double,
			  unsigned int i_nbSteps) : CkwsPathOptimizer ()
    {
      max_nb_optimization_loops = i_nbLoops;
      human_size_ = i_double;
      min_steps_number_ = i_nbSteps;
      step_size_ = human_size_/6;
    }

  } // end of namespace hashoptimizer.
} // end of namespace kws.
