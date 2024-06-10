/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
 *
 * This file is part of SCRIMMAGE.
 *
 *   SCRIMMAGE is free software: you can redistribute it and/or modify it under
 *   the terms of the GNU Lesser General Public License as published by the
 *   Free Software Foundation, either version 3 of the License, or (at your
 *   option) any later version.
 *
 *   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
 *   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 *   License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 * @author Eric Squires <eric.squires@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <ground-target/plugins/autonomy/realisticmotion/realisticmotion.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/proto/State.pb.h>
#include <scrimmage/msgs/Event.pb.h>
#include <scrimmage/proto/ProtoConversions.h>


#include <iostream>
#include <limits>
#include <typeinfo>

using std::cout;
using std::endl;

namespace sc = scrimmage;


REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::realisticmotion,
                realisticmotion_plugin)

namespace scrimmage {
namespace autonomy {

void realisticmotion::init(std::map<std::string, std::string> &params) {
    initial_speed_ = sc::get<double>("initial_speed", params, initial_speed_);

    desired_alt_idx_ = vars_.declare(VariableIO::Type::desired_altitude, VariableIO::Direction::Out);
    desired_speed_idx_ = vars_.declare(VariableIO::Type::desired_speed, VariableIO::Direction::Out);
    desired_heading_idx_ = vars_.declare(VariableIO::Type::desired_heading, VariableIO::Direction::Out);

    // PublisherPtr pub_gen_ents_; is declared in the header file
    pub_gen_ents_ = advertise("GlobalNetwork", "GenerateEntity");
   

}

bool realisticmotion::step_autonomy(double t, double dt) {
    // Find nearest entity on other team. Loop through each contact, calculate
    // distance to entity, save the ID of the entity that is closest.

    
    // for (auto it = contacts_->begin(); it != contacts_->end(); it++) {
    //     // if this contact is not on the same team
    //     if (it->second.id().team_id() != parent_->id().team_id()) {
	// 	    follow_id_ = it->first;       
    //     }
    // }
    
    double min_dist = std::numeric_limits<double>::infinity();
    for (auto it = contacts_->begin(); it != contacts_->end(); it++) {

        // Skip if this contact is on the same team
        if (it->second.id().team_id() == parent_->id().team_id()) {
            continue;
        }

        // Calculate distance to entity
        double dist = (it->second.state()->pos() - state_->pos()).norm();

        if (dist < min_dist) {
            // If this is the minimum distance, save distance and reference to
            // entity
            min_dist = dist;
            follow_id_ = it->first;
        }
    }


    if(contacts_->count(follow_id_)>0){
	    sc::StatePtr ent_state = contacts_->at(follow_id_).state();
	
        // Calculate distance to entity
        double dist = (ent_state->pos() - state_->pos()).norm();
        //angle calculation
        Eigen::Vector3d own_point = state_->pos();
        Eigen::Vector3d opponent_point = ent_state->pos();
        Eigen::Vector3d position_vector =  own_point  - opponent_point;
        double dot_vector =  position_vector.dot(own_point);
        double own_magnitude = own_point.norm();
        // double opponent_magnitude = opponent_point.norm();
        double position_magnitude = position_vector.norm();
        //here
        double cos_angle = dot_vector/(own_magnitude * position_magnitude);
        double ang = acos(cos_angle)*180/3.141592654;
        // cout<<dist<<" "<<ang<<endl;



        if(dist<350 && ang<50){
            if(flag==1){
            // Generate the missile 
            // Create a state for the entity and place it at position (10, 10, 100) with
            // a roll, pitch, and yaw of (0 deg, 45 deg, 45 deg).
            // cout<<"hello"<<endl;
            State s;
            s.pos() << state_->pos()(0), state_->pos()(1), state_->pos()(2)-100;
            s.quat() = scrimmage::Quaternion(0, 0, 0);

            // Create the GenerateEntity message
            auto msg = std::make_shared<Message<scrimmage_msgs::GenerateEntity>>();

            // Copy the new state into the message
            sc::set(msg->data.mutable_state(), s);

            // Set the entity_tag that references the entity to be generated in the
            // mission XML file.
            msg->data.set_entity_tag("gen_straight");
        
            // Publish the GenerateEntity message
            pub_gen_ents_->publish(msg);
            flag = 0;
            //Calculate the required heading to follow the other entity
            double heading = atan2(ent_state->pos()(1) - state_->pos()(1),
                                   ent_state->pos()(0) - state_->pos()(0));
            vars_.output(desired_heading_idx_, heading);

            // Match entity's altitude
            vars_.output(desired_alt_idx_, state_->pos()(2));

            // Maintain speed
            vars_.output(desired_speed_idx_, initial_speed_);
            }
            else{
            //Calculate the required heading to follow the other entity
            double heading = atan2(ent_state->pos()(1) - state_->pos()(1),
                                   ent_state->pos()(0) - state_->pos()(0));
            vars_.output(desired_heading_idx_, heading);

            // Match entity's altitude
            vars_.output(desired_alt_idx_, state_->pos()(2));

            // Maintain speed
            vars_.output(desired_speed_idx_, initial_speed_);
            }
            
                      
        }
        
        else if(flag==0){
            //Calculate the required heading to follow the other entity
            double heading = atan2(0.087488663525924,1);
            vars_.output(desired_heading_idx_, heading);

            // Match entity's altitude
            vars_.output(desired_alt_idx_, state_->pos()(2));

            // Maintain speed
            vars_.output(desired_speed_idx_, initial_speed_);
        }


        else {
			//Calculate the required heading to follow the other entity
            double heading = atan2(ent_state->pos()(1) - state_->pos()(1),
                                   ent_state->pos()(0) - state_->pos()(0));
            vars_.output(desired_heading_idx_, heading);

            // Match entity's altitude
            vars_.output(desired_alt_idx_, state_->pos()(2));

            // Maintain speed
            vars_.output(desired_speed_idx_, initial_speed_);
		}
           


    }
    
    
    return true;
}
} // namespace autonomy
} // namespace scrimmage
