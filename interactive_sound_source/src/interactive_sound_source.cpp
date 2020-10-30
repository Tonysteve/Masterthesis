#include <QtWidgets>
#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_sound_source.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "sound_msgs/GlobalSource.h"
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>


using namespace visualization_msgs;





InteractiveSoundSource::InteractiveSoundSource(){

    loadParameter();

    sub_new_source = this->subscribe("/source_position_update",10,&InteractiveSoundSource::get_update,this);
    server =std::make_unique<interactive_markers::InteractiveMarkerServer>("sound_interactive_marker");
    sub_marker_moved = this->subscribe("/sound_interactive_marker/feedback",10, &InteractiveSoundSource::processFeedback,this);

    //init menu
    init_menu();

    // update information about markers
    pub_update_sound = this->advertise<sound_msgs::GlobalSource>("marker_information_update", 1, true);


}

InteractiveSoundSource::~InteractiveSoundSource(){


}

void InteractiveSoundSource::init_menu(){

    interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert( "signal type" );

    // add signal types to menu
    //check the frist entry
    h_mode_last = menu_handler.insert( sub_menu_handle, signal_type_string[0],boost::bind( &InteractiveSoundSource::signal_type_cb, this, _1 ));
    menu_handler.setCheckState( h_mode_last, interactive_markers::MenuHandler::CHECKED );
    for ( int i=1; i<(sizeof(signal_type_string)/sizeof(signal_type_string[0])); i++ )
    {
      interactive_markers::MenuHandler::EntryHandle temp;
      temp = menu_handler.insert( sub_menu_handle, signal_type_string[i],boost::bind( &InteractiveSoundSource::signal_type_cb, this, _1 ));
      menu_handler.setCheckState( temp, interactive_markers::MenuHandler::UNCHECKED );


    }
    //add option for wall parameter setting
    interactive_markers::MenuHandler::EntryHandle Building_material = menu_handler.insert("Building material");
    //add option for wall1
    interactive_markers::MenuHandler::EntryHandle wall1 = menu_handler.insert(Building_material,"wall1");
    beta_wall1 = menu_handler.insert(wall1,material[0],boost::bind( &InteractiveSoundSource::save_beta, this, _1 ));
    menu_handler.setCheckState( beta_wall1, interactive_markers::MenuHandler::CHECKED );
    for ( int i=1; i<(sizeof(material)/sizeof(material[0])); i++ ){
        interactive_markers::MenuHandler::EntryHandle temp_beta;
        temp_beta = menu_handler.insert(wall1,material[i],boost::bind( &InteractiveSoundSource::save_beta, this, _1 ));
        menu_handler.setCheckState( temp_beta, interactive_markers::MenuHandler::UNCHECKED );
    }
    //add option for wall2
    interactive_markers::MenuHandler::EntryHandle wall2 = menu_handler.insert(Building_material,"wall2");
    beta_wall2 = menu_handler.insert(wall2,material[0],boost::bind( &InteractiveSoundSource::save_beta, this, _1 ));
    menu_handler.setCheckState( beta_wall2, interactive_markers::MenuHandler::CHECKED );
    for ( int i=1; i<(sizeof(material)/sizeof(material[0])); i++ ){
        interactive_markers::MenuHandler::EntryHandle temp_beta;
        temp_beta = menu_handler.insert(wall2,material[i],boost::bind( &InteractiveSoundSource::save_beta, this, _1 ));
        menu_handler.setCheckState( temp_beta, interactive_markers::MenuHandler::UNCHECKED );
    }

    //add option for wall3
    interactive_markers::MenuHandler::EntryHandle wall3 = menu_handler.insert(Building_material,"wall3");
    beta_wall3 = menu_handler.insert(wall3,material[0],boost::bind( &InteractiveSoundSource::save_beta, this, _1 ));
    menu_handler.setCheckState( beta_wall3, interactive_markers::MenuHandler::CHECKED );
    for ( int i=1; i<(sizeof(material)/sizeof(material[0])); i++ ){
        interactive_markers::MenuHandler::EntryHandle temp_beta;
        temp_beta = menu_handler.insert(wall3,material[i],boost::bind( &InteractiveSoundSource::save_beta, this, _1 ));
        menu_handler.setCheckState( temp_beta, interactive_markers::MenuHandler::UNCHECKED );
    }
    //add option for wall4
    interactive_markers::MenuHandler::EntryHandle wall4 = menu_handler.insert(Building_material,"wall4");
    beta_wall4 = menu_handler.insert(wall4,material[0],boost::bind( &InteractiveSoundSource::save_beta, this, _1 ));
    menu_handler.setCheckState( beta_wall4, interactive_markers::MenuHandler::CHECKED );
    for ( int i=1; i<(sizeof(material)/sizeof(material[0])); i++ ){
        interactive_markers::MenuHandler::EntryHandle temp_beta;
        temp_beta = menu_handler.insert(wall4,material[i],boost::bind( &InteractiveSoundSource::save_beta, this, _1 ));
        menu_handler.setCheckState( temp_beta, interactive_markers::MenuHandler::UNCHECKED );
    }

    //add option for celling
    interactive_markers::MenuHandler::EntryHandle celling = menu_handler.insert(Building_material,"celling");
    beta_celling = menu_handler.insert(celling,material[0],boost::bind( &InteractiveSoundSource::save_beta, this, _1 ));
    menu_handler.setCheckState( beta_celling, interactive_markers::MenuHandler::CHECKED );
    for ( int i=1; i<(sizeof(material)/sizeof(material[0])); i++ ){
        interactive_markers::MenuHandler::EntryHandle temp_beta;
        temp_beta = menu_handler.insert(celling,material[i],boost::bind( &InteractiveSoundSource::save_beta, this, _1 ));
        menu_handler.setCheckState( temp_beta, interactive_markers::MenuHandler::UNCHECKED );
    }
    //add option for floor
    interactive_markers::MenuHandler::EntryHandle floor = menu_handler.insert(Building_material,"floor");
    beta_floor = menu_handler.insert(floor,material[0],boost::bind( &InteractiveSoundSource::save_beta, this, _1 ));
    menu_handler.setCheckState( beta_floor, interactive_markers::MenuHandler::CHECKED );
    for ( int i=1; i<(sizeof(material)/sizeof(material[0])); i++ ){
        interactive_markers::MenuHandler::EntryHandle temp_beta;
        temp_beta = menu_handler.insert(floor,material[i],boost::bind( &InteractiveSoundSource::save_beta, this, _1 ));
        menu_handler.setCheckState( temp_beta, interactive_markers::MenuHandler::UNCHECKED );
    }

    //add apply changes option to menu
    menu_handler.insert( "apply changes" ,boost::bind( &InteractiveSoundSource::updateposition, this, _1 ));

    //add delete option to menu
    menu_handler.insert( "delete marker" ,boost::bind( &InteractiveSoundSource::delete_marker, this, _1 ));

//    menu_handler->insert( "snr", &InteractiveSoundSource::processFeedback );
//    menu_handler->setCheckState( h_mode_last, interactive_markers::MenuHandler::UNCHECKED );



}

void InteractiveSoundSource::save_beta(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){

    ROS_INFO("beta_type->menu_entry_id is %u",feedback->menu_entry_id);
    //set all other id's to uncheck
//    ROS_INFO("beta_type->menu_entry_id is %s",material[feedback->menu_entry_id].c_str());

    //set beta
    menu_entry_id = feedback->menu_entry_id;
    //save the data
    //locate the marker, which signal type has changed
//    server->clear();
    for(int i = 0;i < int_marker.size();i++){

        if(feedback->marker_name == int_marker[i].name){

            //find the changed option

            //wall1
            if(feedback->menu_entry_id >= 13 && feedback->menu_entry_id <= 15){
                switch(feedback->menu_entry_id){
                case 13:
                    information_of_markers.wall1[i] = 0.99;
                    break;
                case 14:
                    information_of_markers.wall1[i] = 0.9;
                    break;
                case 15:
                    information_of_markers.wall1[i] = 0.1;
                    break;

                }

                menu_handler.setCheckState( beta_wall1, interactive_markers::MenuHandler::UNCHECKED );
                beta_wall1 = feedback->menu_entry_id;
                menu_handler.setCheckState( beta_wall1, interactive_markers::MenuHandler::CHECKED );
            }
            //wall2
            if(feedback->menu_entry_id >= 17 && feedback->menu_entry_id <= 19){

                switch(feedback->menu_entry_id){
                case 17:
                    information_of_markers.wall2[i] = 0.99;
                    break;
                case 18:
                    information_of_markers.wall2[i] = 0.9;
                    break;
                case 19:
                    information_of_markers.wall2[i] = 0.1;
                    break;

                }

                menu_handler.setCheckState( beta_wall2, interactive_markers::MenuHandler::UNCHECKED );
                beta_wall2 = feedback->menu_entry_id;
                menu_handler.setCheckState( beta_wall2, interactive_markers::MenuHandler::CHECKED );
            }

            //wall3
            if(feedback->menu_entry_id >= 21 && feedback->menu_entry_id <= 23){

                switch(feedback->menu_entry_id){
                case 21:
                    information_of_markers.wall3[i] = 0.99;
                    break;
                case 22:
                    information_of_markers.wall3[i] = 0.9;
                    break;
                case 23:
                    information_of_markers.wall3[i] = 0.1;
                    break;

                }
                menu_handler.setCheckState( beta_wall3, interactive_markers::MenuHandler::UNCHECKED );
                beta_wall3 = feedback->menu_entry_id;
                menu_handler.setCheckState( beta_wall3, interactive_markers::MenuHandler::CHECKED );
            }
            //wall4
            if(feedback->menu_entry_id >= 25 && feedback->menu_entry_id <= 27){

                switch(feedback->menu_entry_id){
                case 25:
                    information_of_markers.wall4[i] = 0.99;
                    break;
                case 26:
                    information_of_markers.wall4[i] = 0.9;
                    break;
                case 27:
                    information_of_markers.wall4[i] = 0.1;
                    break;

                }

                menu_handler.setCheckState( beta_wall4, interactive_markers::MenuHandler::UNCHECKED );
                beta_wall4 = feedback->menu_entry_id;
                menu_handler.setCheckState( beta_wall4, interactive_markers::MenuHandler::CHECKED );
            }

            //celling
            if(feedback->menu_entry_id >= 29 && feedback->menu_entry_id <= 31){

                switch(feedback->menu_entry_id){
                case 29:
                    information_of_markers.celling[i] = 0.99;
                    break;
                case 30:
                    information_of_markers.celling[i] = 0.9;
                    break;
                case 31:
                    information_of_markers.celling[i] = 0.1;
                    break;

                }

                menu_handler.setCheckState( beta_celling, interactive_markers::MenuHandler::UNCHECKED );
                beta_celling = feedback->menu_entry_id;
                menu_handler.setCheckState( beta_celling, interactive_markers::MenuHandler::CHECKED );
            }
            //floor
            if(feedback->menu_entry_id >= 33 && feedback->menu_entry_id <= 35){

                switch(feedback->menu_entry_id){
                case 33:
                    information_of_markers.floor[i] = 0.99;
                    break;
                case 34:
                    information_of_markers.floor[i] = 0.9;
                    break;
                case 35:
                    information_of_markers.floor[i] = 0.1;
                    break;

                }
                menu_handler.setCheckState( beta_floor, interactive_markers::MenuHandler::UNCHECKED );
                beta_floor = feedback->menu_entry_id;
                menu_handler.setCheckState( beta_floor, interactive_markers::MenuHandler::CHECKED );
            }
        }
    }



  //  ROS_INFO("Switching to menu entry #%d", h_mode_last);
    menu_handler.reApply( *server );

    server->applyChanges();
}


void InteractiveSoundSource::loadParameter(){
    this->getParam("/sound_simulation_robot/map_frame_id", map_frame_id);



}

visualization_msgs::InteractiveMarkerControl InteractiveSoundSource::makeBoxControl(visualization_msgs::InteractiveMarker &msg){


    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = false;

    control.markers.push_back( makeBox() );
    msg.controls.push_back(control);
    return msg.controls.back();

}


void InteractiveSoundSource::get_update(const sound_msgs::GlobalSourceConstPtr msg){

    sources = *msg;
//    server->clear();
//    server->applyChanges();
//    information_of_markers.sound_poses.poses.clear();
//    information_of_markers.source_type.clear();
//    information_of_markers.amplitude_scale.clear();
    if(sources.sound_poses.poses.size()>0){
        ROS_INFO("the size of sources is %ld",sources.sound_poses.poses.size());

        geometry_msgs::Point sound_source;

          int m = sources.sound_poses.poses.size()-1;
            sound_source = sources.sound_poses.poses[m].position;
            add_int_marker(m,sound_source);

            server->insert(int_marker[m]);
            menu_handler.apply( *server, int_marker[m].name );
            server->applyChanges();

            // add marker information to information_of_markers
            geometry_msgs::Pose pose;
            pose.position = sound_source;
            information_of_markers.sound_poses.poses.push_back(pose);
            information_of_markers.source_type.push_back(0);
            information_of_markers.amplitude_scale.push_back(1);
            information_of_markers.wall1.push_back(0.8);
            information_of_markers.wall2.push_back(0.8);
            information_of_markers.wall3.push_back(0.8);
            information_of_markers.wall4.push_back(0.8);
            information_of_markers.celling.push_back(0.8);
            information_of_markers.floor.push_back(0.8);

    }

}

visualization_msgs::Marker InteractiveSoundSource::makeBox(){

    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = box_size;
    marker.scale.y = box_size;
    marker.scale.z = box_size;

    marker.color.a = 1.0;
    marker.color.r = 0;
    marker.color.g = 1;
    marker.color.b = 0;

    return marker;

}

void InteractiveSoundSource::add_int_marker( int marker_number,geometry_msgs::Point sound_source){
    visualization_msgs::InteractiveMarker new_int_marker;

    new_int_marker.pose.position = sound_source;
    int_marker.push_back(new_int_marker);
    int_marker[marker_number].header.frame_id = "map";
    int_marker[marker_number].scale =1;
    //    int_marker[marker_number].header.frame_id = map_frame_id;
    int_marker[marker_number].header.stamp=ros::Time::now();

    //insert a box
    makeBoxControl(int_marker[marker_number]);
    int_marker[marker_number].controls[0].interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;
    int_marker[marker_number].controls[0].markers[0].color.r = 0;
    int_marker[marker_number].controls[0].markers[0].color.g = 1;
    int_marker[marker_number].controls[0].markers[0].color.b = 0;

    //give a special name to int_marker[marker_number]
    std::stringstream ss;
    ss << "int_marker "<<marker_number;
    std::string str = ss.str();
    int_marker[marker_number].name = str;


    //add 6dof control to int_marker[marker_number]
    visualization_msgs::InteractiveMarkerControl control;
    control.orientation_mode = InteractiveMarkerControl::INHERIT;
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;


    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker[marker_number].controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;

    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker[marker_number].controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;

    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker[marker_number].controls.push_back(control);

//    control.interaction_mode = InteractiveMarkerControl::BUTTON;
//    control.name = "button_control";

//    control.always_visible = true;
//    int_marker[marker_number].controls.push_back(control);

//    control.interaction_mode = InteractiveMarkerControl::MENU;
//    control.name = "menu_only_control";

//    control.always_visible = true;
//    int_marker[marker_number].controls.push_back(control);

}


void InteractiveSoundSource::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
    std::ostringstream s;
    s << "Feedback from marker '" << feedback->marker_name << "' "
        << " / control '" << feedback->control_name << "'";

    switch ( feedback->event_type )
    {

      case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:

        //check first if it is a new pose or the user just clicked
        //...

        ROS_INFO_STREAM( s.str() << ": new pose"
            << "\nposition = "
            << feedback->pose.position.x
            << ", " << feedback->pose.position.y
            << ", " << feedback->pose.position.z
            << "\nframe: " << feedback->header.frame_id);

        //locate the marker, which position has changed
        for(int i = 0;i < int_marker.size();i++){

            if(feedback->marker_name == int_marker[i].name){
                information_of_markers.sound_poses.poses[i].position = feedback->pose.position;
                int_marker[i].pose.position = feedback->pose.position;
            }
//            ROS_INFO("the position of information_of_markers%d is %f,%f,%f",i,information_of_markers.sound_poses.poses[i].position.x,
//            information_of_markers.sound_poses.poses[i].position.y,information_of_markers.sound_poses.poses[i].position.z);
        }

        break;
    }


//    //save marker information and inform sound simulation ...

//    pub_update_sound.publish(information_of_markers);


}

void InteractiveSoundSource::signal_type_cb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  menu_entry_id = 0;

  ROS_INFO("signal_type->menu_entry_id is %s",signal_type_string[feedback->menu_entry_id-2].c_str());

  menu_entry_id = feedback->menu_entry_id;
  //save the data
  //locate the marker, which signal type has changed
  server->clear();
  for(int i = 0;i < int_marker.size();i++){

      if(feedback->marker_name == int_marker[i].name){
          information_of_markers.source_type[i] = (feedback->menu_entry_id-2);

//          //change the color of marker
//           server->erase(int_marker[i].name);
           std_msgs::ColorRGBA a;
           a = int_marker[i].controls[0].markers[0].color;
           int_marker[i].controls[0].markers[0].color=set_color(feedback->menu_entry_id-2,int_marker[i].controls[0].markers[0].color);


//           server->insert(int_marker[i]);

      }
      server->insert(int_marker[i]);
//      ROS_INFO("the type of information_of_markers %d is %d",i,information_of_markers.source_type[i]);
  }
  server->applyChanges();

  //set all other id's to uncheck


  menu_handler.setCheckState( h_mode_last, interactive_markers::MenuHandler::UNCHECKED );
  h_mode_last = feedback->menu_entry_id;
  menu_handler.setCheckState( h_mode_last, interactive_markers::MenuHandler::CHECKED );

//  ROS_INFO("Switching to menu entry #%d", h_mode_last);
  menu_handler.reApply( *server );

  server->applyChanges();

//  pub_update_sound.publish(information_of_markers);



}
void InteractiveSoundSource::delete_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){

    for(int i = 0;i < int_marker.size();i++){

        if(feedback->marker_name == int_marker[i].name){
            server->erase(int_marker[i].name);
            information_of_markers.sound_poses.poses.erase(information_of_markers.sound_poses.poses.begin()+i);
            information_of_markers.source_type.erase(information_of_markers.source_type.begin()+i);
            int_marker.erase(int_marker.begin()+i);
        }

    }
    server->applyChanges();
    pub_update_sound.publish(information_of_markers);

}
void InteractiveSoundSource::updateposition(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){

        //save marker information and inform sound simulation ...

        pub_update_sound.publish(information_of_markers);

}

std_msgs::ColorRGBA InteractiveSoundSource::set_color(int type, std_msgs::ColorRGBA marker_color){

    switch(type){

        case 0:
            marker_color.r = 0;
            marker_color.g = 1;
            marker_color.b = 0;
            marker_color.a = 1;
            break;

        case 1:
            marker_color.r = 255;
            marker_color.g = 0;
            marker_color.b = 0;
            marker_color.a = 1;
            break;

        case 2:
            marker_color.r = 255;
            marker_color.g = 165;
            marker_color.b = 0;
            marker_color.a = 1;
            break;

        case 3:
            marker_color.r = 0;
            marker_color.g = 0;
            marker_color.b = 0;
            marker_color.a = 1;
            break;

        case 4:
            marker_color.r = 0;
            marker_color.g = 255;
            marker_color.b = 0;
            marker_color.a = 1;
            break;

        case 5:
            marker_color.r = 0;
            marker_color.g = 127;
            marker_color.b = 255;
            marker_color.a = 1;
            break;

        case 6:
            marker_color.r = 0;
            marker_color.g = 0;
            marker_color.b = 255;
            marker_color.a = 1;
            break;

        case 7:
            marker_color.r = 139;
            marker_color.g = 0;
            marker_color.b = 255;
            marker_color.a = 1;
            break;

        case 8:
            marker_color.r = 1;
            marker_color.g = 4;
            marker_color.b = 1;
            marker_color.a = 1;
            break;
    }

    return marker_color;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "InteractiveSoundSource");

  ROS_INFO("Starting interactive sound source");
  InteractiveSoundSource in_sound_source;

  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();

  while(ros::ok()){

  }

}
