// Copyright (C) 2008, 2009 by Antonio El Khoury, CNRS.
//
// This file is part of the kws-hash-optimizer.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef KWS_HASH_OPTIMIZER_DISTANCE_HH
# define KWS_HASH_OPTIMIZER_DISTANCE_HH

#include <vector>

#include <KineoWorks2/kwsDistance.h>

# include <hpp/walkfootplanner/fwd.hh>
# include <hpp/util/portability.hh>

namespace kws
{
  namespace hashoptimizer
  {
    KIT_PREDEF_CLASS (Distance);

    typedef std::vector<double> SpeedVector;

    /**
       \brief Distance that returns a higher cost to lateral and backwards walking.
    */
    class Distance : public CkwsDistance
    {
    public:
      
      virtual ~Distance();
      /**
	 \brief Create an object and return a shared pointer.
      */
      static DistanceShPtr create (const double& i_integrationStep,
				   const double& i_vMinX,
				   const double& i_vMaxX,
				   const double& i_vMaxY);

      /**
	 \brief Get the samled configuration at given step index.
       */
      void
      sampleConfiguration (const CkwsDirectPathShPtr& i_directPath,
			   const unsigned int i_stepsNb,
			   const unsigned int i_stepIndex,
			   CkwsConfig& o_cfg) const;

      /**
	 \brief Compute elementary time cost on the direct path at a
	 given step index, using speed constraint defined in the
	 bounding box frame as follows:

	 --- ellipsis defining the speed constraint in the positive-vf
	 semi-plane = (vf / attVMaxX) * (vf / attVMaxX) + (vlat /
	 attVMaxY) * (vlat / attVMaxY) -1.
	 
	 --- ellipsis defining the speed constraint in the negative-vf
	 semi-plane = (vf / attVMinX) * (vf / attVMinX) + (vlat /
	 attVMaxY) * (vlat / attVMaxY) -1.
       */
      virtual double
      elementaryCost (const CkwsConfig &i_cfg1,
		      const CkwsConfig &i_cfg2,
		      const unsigned int i_stepsNb,
		      const unsigned int i_stepIndex) const;

      /**
	 \brief Same as \sa elementaryCost, but also gives the sampled
	 start configuration and the speed vector.
       */
      virtual double
      elementaryCost (const CkwsConfig &i_cfg1,
		      const CkwsConfig &i_cfg2,
		      const unsigned int i_stepsNb,
		      const unsigned int i_stepIndex,
		      CkwsConfig& o_cfg,
		      SpeedVector& o_vector) const;

      /**
	 \brief Compute time distance between two configurations.
      */
      virtual double distance (const CkwsConfig &i_cfg1,
			       const CkwsConfig &i_cfg2) const;

      /**
	 \brief New cost function that gives a higher weight to lateral movement.
      */
      virtual double cost (const CkwsDirectPathConstShPtr &i_directPath) const;

      /**
	 \brief Cost function that returns total cost of path.
      */
      virtual double cost (const CkwsPathConstShPtr &i_path) const;

      /**
	 \brief Return integration step size value.
       */
      virtual double integrationStep () const;

    protected:
      /**
	 \brief Constructor
      */
      Distance (const double& i_integrationStep,
		const double& i_vMinX,
		const double& i_vMaxX,
		const double& i_vMaxY);
      
      /**
	 \brief Initialization
      */
      ktStatus init(DistanceWkPtr inWeakPtr);

    private:
      /**
	 \brief Store weak pointer to itself.
      */
      DistanceWkPtr attWeakPtr;

      /**
	 \brief Store integration step size.
      */
      double attIntegrationStep;

      /**
	 \brief Store minimum backward speed (positive value).
      */
      double attVMinX;
      
      /**
	 \brief Store maximum frontal speed.
      */
      double attVMaxX;

      /**
	 \brief Store maximum orthogonal speed.
      */
      double attVMaxY;
    };
  } // end of namespace hashoptimizer.
} // end of namespace kws.

#endif //! KWS_HASH_OPTIMIZER_DISTANCE_HH
