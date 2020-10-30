#ifndef SRP_SOUND_SIMULATION_H
#define SRP_SOUND_SIMULATION_H

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <dynamic_reconfigure/server.h>
#include "sound_simulation_robot/SimulationConfig.h"
#include <ros/console.h>
#include "manyears/AudioStream.h"
#include <dynamic_reconfigure/server.h>
#include <cmath>
#include <vector>
#include <stdlib.h>
#include <random>
#include <string>
#include <iostream>
#include <fstream>
#include <AudioFile.h>
#include "std_srvs/Empty.h"
#include "sound_msgs/GlobalSource.h"
#include <Eigen/Dense>

#include <move_base_msgs/MoveBaseAction.h>

#include <random>

class SoundSimulationRobot: public ros::NodeHandle{

private:

    ros::Publisher pub_audio;
    //ros::Publisher pub_sources ;
    ros::Subscriber sub_new_source;
    bool gotNewSource;
    // check the goal position of robot
    ros::Subscriber sub_nav_status;
    bool goalReached;
    bool waitForGoal;

    ros::ServiceServer service_showTime;

    tf::TransformListener *listener =NULL;

    dynamic_reconfigure::Server<sound_simulation_robot::SimulationConfig> *server =NULL;
    dynamic_reconfigure::Server<sound_simulation_robot::SimulationConfig>::CallbackType f;

    sound_msgs::GlobalSource sources;

    std::string map_frame_id;
    std::string mic_frame_id;

    bool use_noise;
    bool use_activity_thresh;

    std::default_random_engine generator;

    double stddev_x;
    double stddev_y;

    double activity_thresh;
    double publish_rate;

    int counterSources;
    int maxCounterSources;

    enum signal_type {SINE_440, SINE_1000,SINE_440_1000,PULSE_25,PULSE_50,WHITE_NOISE, MUSIC1, SPEECH1, SPEECH2 };

    int sample=0;
    int frame_num=0;

    bool use3d;

    double doubleTo16bit;

    //microphone params
    int num_mics;
    std::vector<geometry_msgs::Point> mic_positions;
    std::string audio_topic;
    std::string mic_frame_id_gt;
    bool useGT;

    //speed of sound
    double c ;
    std::vector<int> offset_list;

    AudioFile<float> audioFile;
    std::vector<short> music1;
    std::vector<short> speech1;
    std::vector<short> speech2;



    int sin440_number;
    std::vector<std::vector <short>> sin440vector;
    int sin1000_number;
    std::vector<std::vector <short>> sin1000vector;
    int sin440_1000_number;
    std::vector<std::vector <short>> sin440_1000vector;
    int pulse25_number;
    std::vector<std::vector <short>> pulse25vector;
    int pulse50_number;
    std::vector<std::vector <short>> pulse50vector;
    int music1_number;
    std::vector<std::vector <short>> music1_vector;
    int speech1_number;
    std::vector<std::vector <short>> speech1_vector;
    int speech2_number;
    std::vector<std::vector <short>> speech2_vector;



    std::vector<double> mic_scale;

    #define WHITE_NOISE_LENGTH 48000
    short white_noise[WHITE_NOISE_LENGTH];

    std::linear_congruential_engine<std::uint_fast32_t,48271,0,2147483647> gen=std::minstd_rand();
    std::normal_distribution<double> normal_rand;

    bool publishSame;
    bool publishedFirst;
    manyears::AudioStream same_msg;

    double snr_linear=0.0;

    //methods
    double dist(const geometry_msgs::Point &pos1,const geometry_msgs::Point &pos2);
    void init_noise_buffer();
    short signal(int sample, int signal_type);
    short noise(double &gain);
    double signal_strength(std::vector<short> &signal,int &mic);
    void load_soundfiles(std::string &filename,std::vector<short> &array);
    void transform_mic_position(std::vector<geometry_msgs::Point> &mic_trans);
    void set_config(sound_simulation_robot::SimulationConfig &config,uint32_t level);
    void loadParameter();
    void get_update(const sound_msgs::GlobalSourceConstPtr msg);
    bool deleteSource(const geometry_msgs::Point &p);
    void addNoise(geometry_msgs::Point &p);
    bool printTimeDifferenceGT(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    std::vector<short> Signal_convolution(int signal_type,std::vector<double> TF);
    std::vector<short> convolution(std::vector<short> f, std::vector<double> g);
    void moveBaseStatusCallback(const move_base_msgs::MoveBaseActionResultConstPtr msg);

public:

    SoundSimulationRobot();
    ~SoundSimulationRobot();
    double sample_rate;
    int BUFFER_SIZE;
    void publishAudio();

};


#endif
