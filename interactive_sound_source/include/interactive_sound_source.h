#ifndef INTER_SOUND_SOURCE
#define INTER_SOUND_SOURCE

#include "ros/ros.h"
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include "sound_msgs/GlobalSource.h"

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <math.h>


class InteractiveSoundSource: public ros::NodeHandle{

private:

    std::unique_ptr<interactive_markers::InteractiveMarkerServer> server;

    //subscriber
    ros::Subscriber sub_new_source;
    ros::Subscriber sub_marker_moved;

    //publisher
    ros::Publisher pub_update_sound;

    //data
    enum signal_type {SINE_440, SINE_1000,SINE_440_1000,PULSE_25,PULSE_50,WHITE_NOISE, MUSIC1, SPEECH1, SPEECH2 };
    std::string signal_type_string[9] = {"SINE_440", "SINE_1000","SINE_440_1000","PULSE_25","PULSE_50","WHITE_NOISE"," MUSIC1", "SPEECH1", "SPEECH2" };
    std::string material[3] = {"beton","beton with window","Carpet on beton"};
    sound_msgs::GlobalSource sources;
    interactive_markers::MenuHandler::EntryHandle signal_type;
    interactive_markers::MenuHandler::EntryHandle sub_menu_handle;
    interactive_markers::MenuHandler::EntryHandle h_mode_last;
    interactive_markers::MenuHandler::EntryHandle beta_wall1;
    interactive_markers::MenuHandler::EntryHandle beta_wall2;
    interactive_markers::MenuHandler::EntryHandle beta_wall3;
    interactive_markers::MenuHandler::EntryHandle beta_wall4;
    interactive_markers::MenuHandler::EntryHandle beta_celling;
    interactive_markers::MenuHandler::EntryHandle beta_floor;
    std::vector<visualization_msgs::Marker> box_marker;
    std::vector<visualization_msgs::InteractiveMarker> int_marker;
    int id;
    std::string map_frame_id;

    interactive_markers::MenuHandler menu_handler;
    double box_size = 0.2;
    //add sources to save the information of markers
    sound_msgs::GlobalSource information_of_markers;


    // try to add a global variable menu_entry_id
    int menu_entry_id;

    //methods
    void loadParameter();
    void get_update(const sound_msgs::GlobalSourceConstPtr msg);
    void signal_type_cb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
    void updateposition(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
    void delete_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
    std_msgs::ColorRGBA set_color(int type, std_msgs::ColorRGBA marker_color);
    void init_menu();
    visualization_msgs::Marker makeBox();
    visualization_msgs::InteractiveMarkerControl makeBoxControl(visualization_msgs::InteractiveMarker &marker);
    void add_int_marker( int marker_number,geometry_msgs::Point sound_source);

    void save_beta(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

public:

    InteractiveSoundSource();
    ~InteractiveSoundSource();

};

#endif

