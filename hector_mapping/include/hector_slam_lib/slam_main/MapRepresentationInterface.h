//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef _hectormaprepresentationinterface_h__
#define _hectormaprepresentationinterface_h__

#include <Eigen/Core>
#include "../map/GridMap.h"

class ConcreteOccGridMapUtil;
class MapLockerInterface;

namespace hectorslam{

class MapRepresentationInterface
{
public:

  virtual ~MapRepresentationInterface() {};

  virtual void reset() = 0;

  virtual float getScaleToMap() const = 0;

  virtual int getMapLevels() const = 0;
  virtual const GridMap& getGridMap(int mapLevel) const = 0;


  virtual void addMapMutex(int i, MapLockerInterface* mapMutex) = 0;
  virtual MapLockerInterface* getMapMutex(int i) = 0;

  virtual void onMapUpdated() = 0;

  virtual Eigen::Vector3f matchData(const Eigen::Vector3f& beginEstimateWorld, const DataContainer& dataContainer, Eigen::Matrix3f& covMatrix) = 0;

  virtual void updateByScan(const DataContainer& dataContainer, const Eigen::Vector3f& robotPoseWorld) = 0;

  virtual void setUpdateFactorFree(float free_factor) = 0;
  virtual void setUpdateFactorOccupied(float occupied_factor) = 0;
};

}

#endif
