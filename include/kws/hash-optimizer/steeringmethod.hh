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
 * \brief Declaration of SteeringMethod.
 */

#ifndef KWS_HASH_OPTIMIZER_STEERINGMETHOD_HH_
# define KWS_HASH_OPTIMIZER_STEERINGMETHOD_HH_

#include <KineoWorks2/kwsSteeringMethod.h>

namespace kws
{
  namespace hashoptimizer
  {
    /// \brief This class has been generated automatically
    /// See Doxygen documentation to learn how to document your classes:
    /// http://www.stack.nl/~dimitri/doxygen/

    KIT_PREDEF_CLASS (SteeringMethod);

    class SteeringMethod : public CkwsSteeringMethod
    {
    public:
      static SteeringMethodShPtr create (bool i_oriented = false);
      
      virtual CkwsDirectPathShPtr
      makeDirectPath (const CkwsConfig& i_startCfg,
		      const CkwsConfig& i_endCfg);

      virtual bool
      isOriented () const;

    protected:
      ktStatus
      init (const SteeringMethodWkPtr &i_smWkPtr);

      SteeringMethod (bool i_oriented = false);

    private:
      SteeringMethodWkPtr attWeakPtr_;
    };
  } // end of namespace hashoptimizer.
} // end of namespace kws.

#endif //! KWS_HASH_OPTIMIZER_STEERINGMETHOD_HH_
