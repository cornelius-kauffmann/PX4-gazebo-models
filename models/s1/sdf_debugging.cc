/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "LiftDrag.hh"

#include <algorithm>
#include <string>
#include <vector>
#include <cmath>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <sdf/Element.hh>

#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/Inertial.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ExternalWorldWrenchCmd.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Wind.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::LiftDragPrivate
{
public:
    void Load(const EntityComponentManager &_ecm, const sdf::ElementPtr &_sdf);
    void Update(EntityComponentManager &_ecm);

    Model model{kNullEntity};
    double cla = 1.0;
    double cda = 0.01;
    double cma = 0.0;
    double alphaStall = GZ_PI_2;
    double claStall = 0.0;
    double cdaStall = 1.0;
    double cmaStall = 0.0;
    double cm_delta = 0.0;
    double rho = 1.2041; // Luftdichte kann durch Benutzereingabe überschrieben werden
    bool radialSymmetry = false;
    double area = 1.0;
    double alpha0 = 0.0;
    gz::math::Vector3d cp = math::Vector3d::Zero;
    math::Vector3d forward = math::Vector3d::UnitX;
    math::Vector3d upward = math::Vector3d::UnitZ;
    double controlJointRadToCL = 4.0;
    Entity linkEntity;
    Entity controlJointEntity;
    bool validConfig{false};
    sdf::ElementPtr sdfConfig;
    bool initialized{false};
};

void LiftDragPrivate::Load(const EntityComponentManager &_ecm, const sdf::ElementPtr &_sdf)
{
    this->cla = _sdf->Get<double>("cla", this->cla).first;
    this->cda = _sdf->Get<double>("cda", this->cda).first;
    this->cma = _sdf->Get<double>("cma", this->cma).first;
    this->alphaStall = _sdf->Get<double>("alpha_stall", this->alphaStall).first;
    this->claStall = _sdf->Get<double>("cla_stall", this->claStall).first;
    this->cdaStall = _sdf->Get<double>("cda_stall", this->cdaStall).first;
    this->cmaStall = _sdf->Get<double>("cma_stall", this->cmaStall).first;
    this->rho = _sdf->Get<double>("air_density", this->rho).first;
    this->radialSymmetry = _sdf->Get<bool>("radial_symmetry", this->radialSymmetry).first;
    this->area = _sdf->Get<double>("area", this->area).first;
    this->alpha0 = _sdf->Get<double>("a0", this->alpha0).first;
    this->cp = _sdf->Get<math::Vector3d>("cp", this->cp).first;
    this->cm_delta = _sdf->Get<double>("cm_delta", this->cm_delta).first;

    this->forward = _sdf->Get<math::Vector3d>("forward", this->forward).first;
    this->forward.Normalize();

    this->upward = _sdf->Get<math::Vector3d>("upward", this->upward).first;
    this->upward.Normalize();

    this->controlJointRadToCL = _sdf->Get<double>("control_joint_rad_to_cl", this->controlJointRadToCL).first;

    if (_sdf->HasElement("link_name"))
    {
        sdf::ElementPtr elem = _sdf->GetElement("link_name");
        auto linkName = elem->Get<std::string>();
        auto entities = entitiesFromScopedName(linkName, _ecm, this->model.Entity());

        if (entities.empty())
        {
            gzerr << "Link with name[" << linkName << "] not found. " << "The LiftDrag will not generate forces\n";
            this->validConfig = false;
            return;
        }
        else if (entities.size() > 1)
        {
            gzwarn << "Multiple link entities with name[" << linkName << "] found. " << "Using the first one.\n";
        }

        this->linkEntity = *entities.begin();
        if (!_ecm.EntityHasComponentType(this->linkEntity, components::Link::typeId))
        {
            this->linkEntity = kNullEntity;
            gzerr << "Entity with name[" << linkName << "] is not a link\n";
            this->validConfig = false;
            return;
        }
    }
    else
    {
        gzerr << "The LiftDrag system requires the 'link_name' parameter\n";
        this->validConfig = false;
        return;
    }

    if (_sdf->HasElement("control_joint_name"))
    {
        auto controlJointName = _sdf->Get<std::string>("control_joint_name");
        auto entities = entitiesFromScopedName(controlJointName, _ecm, this->model.Entity());

        if (entities.empty())
        {
            gzerr << "Joint with name[" << controlJointName << "] not found. " << "The LiftDrag will not generate forces\n";
            this->validConfig = false;
            return;
        }
        else if (entities.size() > 1)
        {
            gzwarn << "Multiple joint entities with name[" << controlJointName << "] found. Using the first one.\n";
        }

        this->controlJointEntity = *entities.begin();
        if (!_ecm.EntityHasComponentType(this->controlJointEntity, components::Joint::typeId))
        {
            this->controlJointEntity = kNullEntity;
            gzerr << "Entity with name[" << controlJointName << "] is not a joint\n";
            this->validConfig = false;
            return;
        }
    }

    // If we reached here, we have a valid configuration
    this->validConfig = true;
}

// Der Rest des Codes bleibt unverändert, da hier nur der Ladeprozess geändert wurde.

