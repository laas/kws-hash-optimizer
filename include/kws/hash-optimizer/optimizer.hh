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
            
      /// \brief Get value of the maximum number of loops
      /// max_nb_loop_optimizer_ that the basic optimizer is allowed
      /// to do.
      unsigned int NbOptimizationLoops ();

      /// \brief Get value of the hashing interval step_size_.
      double stepSize ();

      /// \brief Get value of the minimum number of steps
      /// min_steps_number_ in a direct path under which the direct
      /// path will not be hashed.
      unsigned int minStepsNb ();

      /// \brief Get value of the angle lateral_angle_ by which
      /// configuration is rotated to be oriented laterally (+ or - pi
      /// / 2).
      double lateralAngle ();

      /// \brief Get shared pointer to input path to be optimized
      /// i_path_
      CkwsPathShPtr inPath () const;

      /// \brief Get shared pointer to optimized path under
      /// construction o_path_
      CkwsPathShPtr outPath ();

      /// \brief Get shared pointer to direct path collision validator
      /// dp_validator_
      CkwsValidatorDPCollisionShPtr dpValidator () const;

      /// \brief Get shared pointer to direct path collision validator
      /// cfg_validator_
      CkwsValidatorCfgCollisionShPtr cfgValidator () const;

      /// \brief Get original configuration for current step direct
      /// path.
      CkwsConfig originalConfig () const;

      /// \brief Get index of direct path being being currently
      /// optimized.
      unsigned int dpIndex () const;

      /// \brief Get index of direct path step being currently
      /// processed.
      unsigned int stepIndex () const;

      /// \brief Get number of steps in direct path being currently
      /// optimized.
      unsigned int stepsNb () const;

    protected:
      // FIXME {doxygen}
      enum {FRONTAL, LATERAL, ORIGINAL};

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
      virtual ktStatus alignPathConfigs ();

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
      retrieveValidators ();

      // FIXME {doxygen}
      virtual ktStatus appendHashedDP ();

      // FIXME {doxygen}
      virtual ktStatus
      rotateDPEndConfig (CkwsConfig& io_config);

      // FIXME {doxygen}
      virtual ktStatus
      tryOrientFrontalDPEndConfig (CkwsConfig& o_config);

      // FIXME {doxygen}
      virtual ktStatus
      tryOrientLateralDPEndConfig (CkwsConfig& io_reorientedConfig);

      // FIXME {doxygen}
      virtual ktStatus
      tryAppendFrontalLastStepDP (CkwsConfig& io_reorientedConfig);

      // FIXME {doxygen}
      virtual ktStatus
      tryAppendLateralLastStepDP (CkwsConfig& io_reorientedConfig);

      // FIXME {doxygen}
      virtual ktStatus
      nextStepConfig (unsigned int i_orientation,
		      CkwsConfig& o_config);

      // FIXME: doxygen
      virtual ktStatus
      getOriginalConfig (CkwsConfig& o_config);
      
      // FIXME {doxygen}
      virtual ktStatus
      adjustOriginalConfig (const CkwsConfig& i_endConfig,
			    CkwsConfig& io_config);

      // FIXME {doxygen}
      virtual ktStatus
      tryMakeStepDP (const CkwsConfig& i_beginConfig,
		     const CkwsConfig& i_endConfig);

      // FIXME {doxygen}
      virtual ktStatus
      appendStepDP ();

      // FIXME: doxygen
      virtual ktStatus
      tryLateralStepConfig (CkwsConfig& io_reorientedConfig);

      // FIXME: doxygen      
      virtual ktStatus
      tryOriginalStepConfig (CkwsConfig& io_reorientedConfig);

      // FIXME: doxygen
      virtual ktStatus
      tryAppendFrontalStepDP (CkwsConfig& io_reorientedCfg);

      // FIXME: doxygen
      virtual ktStatus
      tryAppendLateralStepDP (CkwsConfig& io_reorientedCfg);

      // FIXME: doxygen
      virtual ktStatus
      tryPreviousLateralStepConfig (CkwsConfig& io_reorientedConfig);

      // FIXME: doxygen
      virtual ktStatus
      tryPreviousOriginalStepConfig (CkwsConfig& io_reorientedConfig);

      // FIXME: doxygen
      virtual ktStatus
      tryAppendOriginalStepDP (CkwsConfig& io_reorientedCfg);

      // FIXME: doxygen
      virtual ktStatus appendLastStepDP ();
      
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

      double lateral_angle_;

      CkwsPathShPtr i_path_;

      CkwsPathShPtr o_path_;
      
      CkwsValidatorDPCollisionShPtr dp_validator_;
      
      CkwsValidatorCfgCollisionShPtr cfg_validator_;

      CkwsConfigShPtr original_config_;

      unsigned int dp_index_;

      unsigned int step_index_;

      unsigned int steps_number_;

      OptimizerWkPtr optimizer_;
    };
  } // end of namespace hashoptimizer.
} // end of namespace kws.

#endif //! KWS_HASH_OPTIMIZER_OPTIMIZER_HH_
