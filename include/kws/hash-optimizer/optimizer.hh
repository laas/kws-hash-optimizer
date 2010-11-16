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
 * \brief Declaration of Optimizer.
 */

#ifndef KWS_HASH_OPTIMIZER_OPTIMIZER_HH_
# define KWS_HASH_OPTIMIZER_OPTIMIZER_HH_

#include "KineoWorks2/kwsPathOptimizer.h"

namespace kws
{
  namespace hashoptimizer
  {
    /// \brief This class has been generated automatically
    /// See Doxygen documentation to learn how to document your classes:
    /// http://www.stack.nl/~dimitri/doxygen/

    KIT_PREDEF_CLASS (Optimizer);

    class Optimizer : public CkwsPathOptimizer
    {
    public:
      virtual ktStatus init (const OptimizerWkPtr& i_weakPtr);

      /// \sa Optimizer ()
      static OptimizerShPtr create (unsigned int i_nbLoops,
				    double i_double,
				    unsigned int i_nbSteps);

      virtual ~Optimizer ();
            
      /// \brief Get value of max_nb_loop_optimizer_
      unsigned int NbOptimizationLoops ();

      /// \brief Get value of step_size_.
      double stepSize ();

      /// \brief Get value of min_steps_number_
      unsigned int minStepsNb ();

    protected:
      // FIXME {doxygen}
      enum {FRONTAL, LATERAL, LATERAL_ADJUSTED, ORIGINAL};

      /// \brief Non-atomic implementation of Hash Optimizer
      ///
      /// // FIXME {description}
      ///
      /// \param io_path Input piecewise linear path. Optimized path
      /// is then put here, can be null if optimization fails.
      ///
      /// \return KD_OK | KD_ERROR
      virtual ktStatus doOptimizePath (const CkwsPathShPtr& io_path);

      // FIXME {doxygen}
      virtual ktStatus alignPathConfigs (const CkwsPathShPtr& i_path,
					 CkwsPathShPtr& o_path);

      /// \brief retrieves collision validators from input path.
      ///
      /// \param i_path Input path.
      ///
      /// \retval o_dpValidator Output direct path collision validator
      ///
      /// \retval o_cfgValidator Output configuration collision validator
      ///
      /// \return KD_OK | KD_ERROR
      virtual ktStatus
      retrieveValidators (const CkwsPathShPtr& i_path,
			  CkwsValidatorDPCollisionShPtr& o_dpValidator,
			  CkwsValidatorCfgCollisionShPtr& o_cfgValidator);

      // FIXME {doxygen}
      virtual ktStatus
      appendHashedDP (const CkwsPathShPtr& i_path,
		      const CkwsValidatorDPCollisionShPtr& i_dpValidator,
		      const CkwsValidatorCfgCollisionShPtr& i_cfgValidator,
		      unsigned int i_int,
		      CkwsPathShPtr& io_path);

      // FIXME {doxygen}
      virtual ktStatus
      rotateDPEndConfig (const CkwsPathShPtr& i_path,
			 unsigned int i_int,
			 CkwsConfig& io_config);

      // FIXME {doxygen}
      virtual ktStatus
      reorientDPEndConfig (const CkwsPathShPtr& i_path,
			  const CkwsValidatorDPCollisionShPtr&
			  i_dpValidator,
			  const CkwsValidatorCfgCollisionShPtr&
			  i_cfgValidator,
			  unsigned int i_int,
			  CkwsConfig& o_config);

      // FIXME {doxygen}
      virtual ktStatus
      tryOrthogonalDPEndConfig (const CkwsPathShPtr& i_path,
				const CkwsConfig& i_originalConfig,
				const CkwsConfig& i_nextDPEndConfig,
				const CkwsValidatorDPCollisionShPtr&
				i_dpValidator,
				const CkwsValidatorCfgCollisionShPtr&
				i_cfgValidator,
				unsigned int i_int,
				CkwsConfig& io_reorientedConfig);

      // FIXME {doxygen}
      virtual ktStatus
      tryMakeFrontalDPForEndConfig (const CkwsPathShPtr& i_path,
				    const CkwsConfig& i_originalConfig,
				    const CkwsConfig& i_nextDPEndConfig,
				    const CkwsValidatorDPCollisionShPtr&
				    i_dpValidator,
				    const CkwsValidatorCfgCollisionShPtr&
				    i_cfgValidator,
				    unsigned int i_int,
				    CkwsConfig& io_reorientedConfig);

      // FIXME {doxygen}
      virtual ktStatus
      tryMakeOrthogonalDPForEndConfig (const CkwsPathShPtr& i_path,
				       const CkwsConfig& i_originalConfig,
				       const CkwsConfig& i_nextDPEndConfig,
				       const CkwsValidatorDPCollisionShPtr&
				       i_dpValidator,
				       const CkwsValidatorCfgCollisionShPtr&
				       i_cfgValidator,
				       unsigned int i_int,
				       CkwsConfig& io_reorientedConfig);

      // FIXME: doxygen
      virtual ktStatus getOriginalConfig (const CkwsPathShPtr& i_path,
					  unsigned int i_dpIndexInt,
					  unsigned int i_stepIndexInt,
					  unsigned int i_stepNumberInt,
					  CkwsConfig& o_config);	

      // FIXME {doxygen}
      virtual ktStatus nextStepConfig (const CkwsConfig& i_beginConfig,
				       const CkwsConfig& i_endConfig,
				       const CkwsConfig& i_nextDPEndConfig,
				       unsigned int i_orientation,
				       const CkwsValidatorCfgCollisionShPtr&
				       i_cfgValidator,
				       CkwsConfig& o_config);
      
      // FIXME {doxygen}
      virtual ktStatus
      adjustLateralConfig (const CkwsConfig& i_endConfig,
			   const CkwsConfig& i_nextDPEndConfig,
			   CkwsConfig& io_config);

      // FIXME {doxygen}
      virtual ktStatus
      adjustOriginalConfig (const CkwsConfig& i_endConfig,
			    CkwsConfig& io_config);

      // FIXME {doxygen}
      virtual ktStatus
      tryMakeStepDP (const CkwsConfig& i_beginConfig,
		     const CkwsConfig& i_endConfig,
		     const CkwsValidatorDPCollisionShPtr& i_dpValidator);
	
      // FIXME {doxygen}
      virtual ktStatus
      appendStepDP (const CkwsConfig& i_dpEndConfig,
		    const CkwsConfig& i_nextDPEndConfig,
		    const CkwsValidatorDPCollisionShPtr&
		    i_dpValidator,
		    const CkwsValidatorCfgCollisionShPtr&
		    i_cfgValidator,
		    unsigned int i_int,
		    const unsigned int i_nbSteps,
		    CkwsConfig& io_lastConfig,
		    CkwsPathShPtr& io_path);

      // FIXME: doxygen
      virtual ktStatus
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
			    CkwsPathShPtr& io_path);

      // FIXME: doxygen      
      virtual ktStatus
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
			     CkwsPathShPtr& io_path);

      // FIXME: doxygen
      virtual ktStatus
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
			      CkwsPathShPtr& io_path);

      // FIXME: doxygen
      virtual ktStatus
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
			      CkwsPathShPtr& io_path);

      // FIXME: doxygen
      virtual ktStatus
      tryPreviousLateralStepConfig (const CkwsConfig& i_dpEndConfig,
				    const CkwsConfig i_nextDPEndConfig,
				    const CkwsValidatorDPCollisionShPtr&
				    i_dpValidator,
				    const CkwsValidatorCfgCollisionShPtr&
				    i_cfgValidator,
				    unsigned int i_int,
				    const unsigned int i_nbSteps,
				    CkwsConfig& io_lastConfig,
				    CkwsConfig& io_reorientedConfig,
				    CkwsPathShPtr& io_path);
 
      // FIXME: doxygen
      virtual ktStatus
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
			       CkwsPathShPtr& io_path);

      /// \brief Constructor
      ///
      /// \param i_nbLoops Maximum number of loops to be run by
      /// Adaptive Shorcut Optimizer.
      ///
      /// \param i_double Human size.
      Optimizer (unsigned int i_nbLoops,
		 double i_double,
		 unsigned int i_nbSteps);

    private:
      unsigned int max_nb_optimization_loops;

      double human_size_;

      double step_size_;
      
      unsigned int min_steps_number_;

      OptimizerWkPtr optimizer_;
    };
  } // end of namespace hashoptimizer.
} // end of namespace kws.

#endif //! KWS_HASH_OPTIMIZER_OPTIMIZER_HH_
