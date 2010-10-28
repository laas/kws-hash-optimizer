// Copyright (C) 2010 by Antonio El Khoury.
//
// This file is part of the kws-hash-optimizer.
//
// kws-hash-optimizer is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// kws-hash-optimizer is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with kws-hash-optimizer.  If not, see <http://www.gnu.org/licenses/>.

/**
 * \brief Forward declarations.
 */

#ifndef KWS_HASH_OPTIMIZER_IO_HH
# define KWS_HASH_OPTIMIZER_IO_HH

namespace kws
{
  namespace hashoptimizer
  {
    std::ostream& operator<< (std::ostream& i_string, const Optimizer& i_optimizer)
    {
      i_optimizer.display (i_string);
      return i_string;
    }
  } // end of namespace hashoptimizer
} // end of namespace kws 

#endif //! KWS_HASH_OPTIMIZER_IO_HH
