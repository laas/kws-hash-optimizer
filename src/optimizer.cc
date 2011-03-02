// Copyright (C) 2010-2011 by Antonio El Khoury.
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
#include <KineoWorks2/kwsRandomOptimizer.h>
#include <KineoWorks2/kwsRoadmap.h>
#include <KineoWorks2/kwsNode.h>
#include <KineoWorks2/kwsEdge.h>

//FIXME: update later hpp-util in robotpkg to include sstream
#include <sstream>
#include <hpp/util/debug.hh>
#include <hpp/util/indent.hh>

#include "kws/hash-optimizer/directpath.hh"
#include "kws/hash-optimizer/steeringmethod.hh"
#include "kws/hash-optimizer/distance.hh"
#include "kws/hash-optimizer/elliptic-steeringmethod.hh"
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

    double Optimizer::stepSize () const
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
      attDistance = KIT_DYNAMIC_PTR_CAST(Distance const, distance ());

      // Optimize path first with random optimizer.
      CkwsLoopOptimizerShPtr randomOptimizer
	= CkwsRandomOptimizer::create ();
      randomOptimizer->maxNbLoop (NbOptimizationLoops ());
      randomOptimizer->minGainStop (0.0);

      if (NbOptimizationLoops () != 0)
	if (KD_ERROR == randomOptimizer->optimizePath (io_path))
	  {
	    hppDout(error, "Random optimization could not be completed");
	    return KD_ERROR;
	  }
      
      // Rebuild randomly optimized path with direct path that take only
      // translation position variables in interpolation and length
      // computation.
      // FIXME: make it work for B-spline paths as well?
      CkwsConfig dpStartCfg (device ());
      CkwsConfig dpEndCfg (device ());
      CkwsSteeringMethodShPtr steeringMethod = SteeringMethod::create ();
      i_path_ = CkwsPath::create (device ());
      for (unsigned int i = 0; i < io_path->countConfigurations () - 1; i++)
	{
	  io_path->getConfiguration (i, dpStartCfg);
	  io_path->getConfiguration (i + 1, dpEndCfg);

	  CkwsDirectPathShPtr directPath = 
	    steeringMethod->makeDirectPath (dpStartCfg, dpEndCfg);

	  i_path_->appendDirectPath (directPath);
	}
      attLength = inPath ()->length ();

      // Start hash optimization.
      o_path_ = CkwsPath::create (device ());
      CkwsConfig startCfg (device ());
      inPath ()->getConfigAtStart (startCfg);
      outPath ()->setInitialConfig (startCfg);
      original_config_ = CkwsConfig::create (device ());
      retrieveValidators ();

      if (KD_ERROR == aStar (inPath ()))
	{
	  hppDout(error, "Hash optimization could not be completed");
	}
      
      *io_path = *outPath ();

      return KD_OK;
    }

    ktStatus Optimizer::aStar (const CkwsPathShPtr& i_path)
    {
      // Variables initialization.
      NodeAndCostSet closedSet;
      NodeAndCostSet openSet;

      CkwsConfig startCfg (device ());
      i_path->getConfigAtStart (startCfg);
      CkwsNodeShPtr startNode = CkwsNode::create (startCfg);

      CkwsConfig endCfg (device ());
      i_path->getConfigAtEnd (endCfg);
      CkwsNodeShPtr endNode = CkwsNode::create (endCfg);
      
      NodeAndDistance startNodeAndDistance (startNode, 0.);
     
      double g = 0.;
      double h = heuristicEstimate (NodeAndCost (startNodeAndDistance,
						 Cost (0., 0., 0.)),
				    startNodeAndDistance,
				    i_path);
      double f = h;
      
      Cost startCost (g, h, f);
      openSet.insert (NodeAndCost (startNodeAndDistance, startCost));
      
      CkwsRoadmapShPtr graph = CkwsRoadmap::create (device ());
      graph->addNode (startNode);
      graph->addNode (endNode);

      // Run A* search.
      while (!openSet.empty ())
      	{
      	  NodeAndCost pair = bestPair (openSet);

	  NodeAndDistance nodeAndDistance = pair.first;
      	  CkwsNodeShPtr node = nodeAndDistance.first;
      	  double g = boost::get<0> (pair.second);

	  // hppDout (notice, "openset:");
	  // for (NodeAndCostSet::iterator nodeIt = openSet.begin ();
      	  //      nodeIt != openSet.end ();
      	  //      nodeIt++)
	  //   {
	  //     hppDout (notice, ((*nodeIt).first).first->config ().dofValue (0) << "\t"
	  // 	       << ((*nodeIt).first).first->config ().dofValue (1) << "\t"
	  // 	       << ((*nodeIt).first).first->config ().dofValue (5));
	  //   }
	  
	  // hppDout (notice, "best g " << g);
	  // hppDout (notice, "best configuration " << incindent << iendl
	  // 	   << (pair.first).first->config ().dofValue (0) << iendl
	  // 	   << (pair.first).first->config ().dofValue (1) << iendl
	  // 	   << (pair.first).first->config ().dofValue (5)
	  // 	   << decindent << iendl
	  // 	   << "evaluation functions " << incindent << iendl
	  // 	   << g << iendl << h << iendl << f
	  // 	   << "Roadmap edges nb " << graph->countEdges ()
	  // 	   << decindent << iendl);
	  
	  // showRoadmapCC (graph);
	  
	  // std::cin.get ();

      	  if (node->config () == endNode->config ())
      	    {
	      std::list<CkwsEdgeShPtr> edges;
	      
      	      bool pathFound = graph->findPath (startNode, endNode, attDistance,
      						o_path_, edges);
      	      if (pathFound)
      		return KD_OK;
      	      else
      		{
      		  hppDout (error, "Search completed but no path was found.");
      		  return KD_ERROR;
      		}
      	    }

      	  openSet.erase (pair.first);
      	  closedSet.insert (pair);

      	  NodeAndCostSet neighborNodes;
	  expand (pair, i_path, graph, neighborNodes);

	  // Update evaluation function values.
      	  for (NodeAndCostSet::iterator nodeIt = neighborNodes.begin ();
      	       nodeIt != neighborNodes.end ();
      	       nodeIt++)
      	    {
      	      NodeAndCost loopNodeAndCost = *nodeIt;
	      bool isBetter;

	      if (findInSet (loopNodeAndCost, closedSet) != closedSet.end ())
		continue;
	      
	      const double newG = g
		+ nodeDistance (node, (loopNodeAndCost.first).first);
	      
	      NodeAndCostSet::iterator nodeAndCostIt
		= findInSet (loopNodeAndCost, openSet);

      	      if (nodeAndCostIt == openSet.end ())
      	      	{
      	      	  openSet.insert (loopNodeAndCost);
		  isBetter = true;
      	      	}
	      else 
		{
		  double loopG = boost::get<0> ((*nodeAndCostIt).second);
		  
		  if (newG < loopG)
		    isBetter = true;
		  else
		    isBetter = false;
		  
		  if (isBetter)
		    {
		      double newH = heuristicEstimate (pair,
						       (*nodeAndCostIt).first,
						       i_path);
		      double newF = newG + newH;
		      
		      openSet.erase ((*nodeAndCostIt).first);
		      openSet.insert (NodeAndCost ((*nodeAndCostIt).first,
						   Cost (newG, newH, newF)));
		    }
		}
      	    }
      	}

      return KD_ERROR;
    }

    NodeAndCostSet::iterator Optimizer::
    findInSet (const NodeAndCost& i_nodeAndCost,
	       NodeAndCostSet& i_nodeAndCostSet) const
    {
      for (NodeAndCostSet::iterator nodeIt = i_nodeAndCostSet.begin ();
	   nodeIt != i_nodeAndCostSet.end ();
	   nodeIt++)
	{
	  if ((*nodeIt).first.first->config ()
	      == i_nodeAndCost.first.first->config ())
	    {
	      return nodeIt;
	    }
	}

      return i_nodeAndCostSet.end ();
    }

    void Optimizer::
    showRoadmapNodes (const CkwsRoadmapShPtr& i_roadmap) const
    {
      hppDout (notice, "Roadmap node configurations: " << incindent);

      for (unsigned int rank = 0; rank < i_roadmap->countNodes (); rank++)
	{
	  CkwsConfig nodeCfg = i_roadmap->node (rank)->config ();
	  
	  hppDout (notice, rank << "\t" << nodeCfg.dofValue (0) << "\t"
		   << nodeCfg.dofValue (1) << "\t"<< nodeCfg.dofValue (5));
	}
    }

    void Optimizer::
    showRoadmapEdges (const CkwsRoadmapShPtr& i_roadmap) const
    {
      hppDout (notice, "Roadmap nodes: " << incindent);

      for (unsigned int nodeRank = 0;
	   nodeRank < i_roadmap->countNodes ();
	   nodeRank++)
	{
	  hppDout (notice, "Roadmap node edges: " << incindent);
		
	  CkwsNodeShPtr node = i_roadmap->node (nodeRank);

	  for (unsigned int edgeRank = 0;
	       edgeRank < node->countOutEdges ();
	       edgeRank++)
	    {
	      CkwsConfig nodeCfg
		= node->outEdge (edgeRank)->endNode ()->config ();
	      
	      hppDout (notice, edgeRank << "\t" << nodeCfg.dofValue (0) << "\t"
		   << nodeCfg.dofValue (1) << "\t"<< nodeCfg.dofValue (5));
	    }
	}
    }

    void Optimizer::
    showRoadmapCC (const CkwsRoadmapShPtr& i_roadmap) const
    {
      hppDout (notice, "Roadmap connected components " << incindent);
      
      for (unsigned int rank = 0;
	   rank < i_roadmap->countConnectedComponents (); rank++)
	{
	  unsigned int nodeNb
	    = i_roadmap->connectedComponent (rank)->countNodes ();
	  
	  hppDout (notice, rank << "\t" << nodeNb);
	}
    }

    double Optimizer::
    heuristicEstimate (const NodeAndCost& i_nodeAndCost1,
		       const NodeAndDistance& i_nodeAndDistance2,
		       const CkwsPathShPtr& i_path)
    {
      double distance2 = i_nodeAndDistance2.second;
      
      if (distance2 == attLength)
	return 0.;

      const double distance3 = distance2 + stepSize ();
      
      const CkwsConfig cfg1 = i_nodeAndCost1.first.first->config ();
      const CkwsConfig cfg2 = i_nodeAndDistance2.first->config ();

      CkwsConfig frontalCfg2 (cfg2);
      makeTangentConfig (frontalCfg2, distance2, i_path);

      double heuristicEstimate2 = 0.;

      if (i_nodeAndDistance2.second == 0.)
	{
	  CkwsConfig endCfg (device ());
	  i_path->getConfigAtEnd (endCfg);

	  if (distance3 < attLength)
	    {
	      CkwsConfig frontalCfg (device ());
	      i_path->getConfigAtDistance (distance3, frontalCfg);
	      makeTangentConfig (frontalCfg, distance3, i_path);
	      
	      heuristicEstimate2 = attDistance->distance (cfg2, frontalCfg);

	      double nextDistance = distance3 + stepSize ();
	      
	      while (nextDistance < attLength)
		{
		  CkwsConfig nextFrontalCfg (device ());
		  i_path->getConfigAtDistance (nextDistance, nextFrontalCfg);
		  makeTangentConfig (nextFrontalCfg, nextDistance, i_path);

		  heuristicEstimate2
		    += attDistance->distance (frontalCfg, nextFrontalCfg);

		  frontalCfg = nextFrontalCfg;

		  nextDistance +=stepSize ();
		}

	      heuristicEstimate2 += attDistance->distance (frontalCfg, endCfg);
	    }
	  else
	    {
	      heuristicEstimate2 = attDistance->distance (cfg2, endCfg);
	    }
	}
      else
	{
	  const double heuristicEstimate1
	    = boost::get<1> (i_nodeAndCost1.second);
	  
	  heuristicEstimate2 = heuristicEstimate1
	    - attDistance->distance (cfg1, frontalCfg2);

	  if (distance3 < attLength)
	    {
	      CkwsConfig frontalCfg3 (device ());
	      i_path->getConfigAtDistance (distance3, frontalCfg3);
	      makeTangentConfig (frontalCfg3, distance3, i_path);

	      heuristicEstimate2 
		+= - attDistance->distance (frontalCfg2, frontalCfg3)
		+ attDistance->distance (cfg2, frontalCfg3);
	    }
	}
      
      return heuristicEstimate2;
    }

    NodeAndCost Optimizer::bestPair (NodeAndCostSet& i_set) const 
    {
      NodeAndCostSet::iterator bestIt = i_set.begin ();
      double minF = boost::get <2> ((*bestIt).second);
      
      for (NodeAndCostSet::iterator it = i_set.begin ();
	   it != i_set.end ();
	   it++)
	{
	  double currentF = boost::get <2> ((*it).second);
	  
	  if (currentF < minF)
	    {
	      bestIt = it;
	      minF = currentF;
	    }
	}
      
      return *(bestIt);
    }

    ktStatus Optimizer::
    expand (const NodeAndCost& i_nodeAndCost,
	    const CkwsPathShPtr& i_path,
	    CkwsRoadmapShPtr& io_graph,
	    NodeAndCostSet& o_set)
    {
      double tryDistance = (i_nodeAndCost.first).second + stepSize ();

      double distance = tryDistance + stepSize () < attLength ?
	tryDistance :
	attLength;

      CkwsConfig sampleCfg (device ());
      i_path->getConfigAtDistance (distance, sampleCfg);
      
      CkwsConfig previousCfg (((i_nodeAndCost.first).first)->config ());

      CkwsSteeringMethodShPtr steeringMethod
	= EllipticSteeringMethod::create (attDistance);
      CkwsDirectPathShPtr directPath 
	= steeringMethod->makeDirectPath (previousCfg, sampleCfg);
      dpValidator ()->validate (*directPath);

      if (directPath->isValid ())
	{
	  CkwsNodeShPtr sampleNode;
	  if (distance == attLength)
	    sampleNode = io_graph->nodeWithConfig (sampleCfg);
	  else
	    {
	      sampleNode = CkwsNode::create (sampleCfg);
	      io_graph->addNode (sampleNode);
	    }
	 
	  const CkwsEdgeShPtr edge = CkwsEdge::create (directPath);
	  io_graph->addEdge ((i_nodeAndCost.first).first,
			     sampleNode,
			     edge);

	  const CkwsEdgeShPtr reverseEdge
	    = CkwsEdge::create (CkwsDirectPath::createReversed (directPath));
	  io_graph->addEdge (sampleNode,
			     (i_nodeAndCost.first).first,
			     reverseEdge);
	 
  	  const NodeAndDistance sampleNodeAndDistance (sampleNode, distance);

	  const double previousG = boost::get<0> (i_nodeAndCost.second);
	  double sampleG = previousG
	    + attDistance->distance (previousCfg, sampleCfg);

	  const double sampleH
	    = heuristicEstimate (i_nodeAndCost, sampleNodeAndDistance, i_path);
	  
	  const double sampleF = sampleG + sampleH;

	  const Cost sampleCost (sampleG, sampleH, sampleF);

	  o_set.insert (NodeAndCost (sampleNodeAndDistance, sampleCost));
	}
      
      CkwsConfig copySampleCfg (sampleCfg);
      makeTangentConfig (copySampleCfg, distance, i_path);

      for (unsigned int i = 0; i < attOrientationAngles.size (); i++)
	{
	  CkwsConfig orientedCfg (copySampleCfg);
	  orientedCfg.dofValue (5, orientedCfg.dofValue (5)
				+ attOrientationAngles.find (i)->second);
	  
	  if (orientedCfg.dofValue (5) == sampleCfg.dofValue (5))
	    continue;

	  CkwsConfig symmetricCfg (i_nodeAndCost.first.first->config ());
	  symmetricCfg.dofValue (5, symmetricCfg.dofValue (5) + M_PI);
	  
	  if (symmetricCfg.isEquivalent (orientedCfg))
	    {
	      hppDout (notice, "configurations are equivalent");
	      continue;
	    }

	  cfgValidator ()->validate (orientedCfg);
	  
	  if (orientedCfg.isValid ())
	    {
	      directPath
		= steeringMethod->makeDirectPath (previousCfg, orientedCfg);
	      
	      dpValidator ()->validate (*directPath);

	      if (directPath->isValid ())
		{
		  CkwsNodeShPtr orientedNode = CkwsNode::create (orientedCfg);
		  io_graph->addNode (orientedNode);

		  CkwsEdgeShPtr edge = CkwsEdge::create (directPath);
		  io_graph->addEdge ((i_nodeAndCost.first).first,
		  		     orientedNode,
		  		     edge);

		  const CkwsEdgeShPtr reverseEdge
		    = CkwsEdge::create (CkwsDirectPath::
					createReversed (directPath));
		  io_graph->addEdge (orientedNode,
				     (i_nodeAndCost.first).first,
				     reverseEdge);

		  const NodeAndDistance orientedNodeAndDistance (orientedNode,
								 distance);
	 
		  const double previousG = boost::get<0> (i_nodeAndCost.second);
		  double orientedG = previousG
		    + attDistance->distance (previousCfg, orientedCfg);
		  
		  const double orientedH
		    = heuristicEstimate (i_nodeAndCost, orientedNodeAndDistance,
					 i_path);
		  
		  const double orientedF = orientedG + orientedH;

		  const Cost orientedCost (orientedG, orientedH, orientedF);
		  
		  o_set.insert (NodeAndCost (orientedNodeAndDistance,
					     orientedCost));
		}
	    }
	}
      
      return KD_OK;
    }

    double Optimizer::nodeDistance (const CkwsNodeShPtr& i_node1,
				    const CkwsNodeShPtr& i_node2) const
    {
      CkwsConfig cfg1 = i_node1->config ();
      CkwsConfig cfg2 = i_node2->config ();

      return attDistance->distance (cfg1, cfg2);
    }

    ktStatus Optimizer::makeTangentConfig (CkwsConfig& io_cfg,
					   const double distance,
					   const CkwsPathShPtr& i_path)
    {
      CkwsConfig elementaryCfg (device ());
      i_path->getConfigAtDistance (distance + stepSize () / 100, elementaryCfg);

      const double dX = elementaryCfg.dofValue (0) - io_cfg.dofValue (0);
      const double dY = elementaryCfg.dofValue (1) - io_cfg.dofValue (1);
      const double angle = atan2 (dY, dX);

      io_cfg.dofValue (5, angle);

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

    Optimizer::Optimizer (unsigned int i_nbLoops,
			  double i_double,
			  unsigned int i_nbSteps) : CkwsPathOptimizer ()
    {
      max_nb_optimization_loops = i_nbLoops;
      human_size_ = i_double;
      min_steps_number_ = i_nbSteps;
      step_size_ = human_size_/6;
      lateral_angle_ = M_PI / 2;

      attOrientationAngles[FRONTAL] = 0;
      attOrientationAngles[LATERAL_1] = M_PI / 2;
      attOrientationAngles[LATERAL_2] = - M_PI / 2;
    }

  } // end of namespace hashoptimizer.
} // end of namespace kws.
