#include <sound_simulation_robot.h>
#include <ros/package.h>
#include <Image_source.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
//#include <sound_simulation_robot/SoundSource.h>
#include <move_base_msgs/MoveBaseAction.h>

SoundSimulationRobot::SoundSimulationRobot(): ros::NodeHandle(){

    loadParameter();

    pub_audio = this->advertise<manyears::AudioStream>(audio_topic,1000);
    sub_new_source = this->subscribe("/marker_information_update", 10, &SoundSimulationRobot::get_update, this);
    //check the status of goal position.
    sub_nav_status = this->subscribe("/move_base/result", 10, &SoundSimulationRobot::moveBaseStatusCallback, this);
    //server
    service_showTime =  this->advertiseService("showTimeDifGT", &SoundSimulationRobot::printTimeDifferenceGT,this);
    listener = new tf::TransformListener();
    counterSources = 0;
    maxCounterSources = 10;

    //wait till server are ready
    ros::Duration d(0.5);
    d.sleep();

    server = new dynamic_reconfigure::Server<sound_simulation_robot::SimulationConfig>();
    f=boost::bind(&SoundSimulationRobot::set_config,this,_1,_2);
    server->setCallback(f);
}

SoundSimulationRobot::~SoundSimulationRobot(){

    printf("Deleting Pointer\n");
    delete listener;
    delete server;

    listener = NULL;
    server = NULL;

}

void SoundSimulationRobot::loadParameter(){
    this->getParam("/sound_simulation_robot/map_frame_id", map_frame_id);
    this->getParam("/microphone_config/frame_id", mic_frame_id);

    //parameter for Sound Sources
    gotNewSource = false;
    //speed of sound
    c = 343;

    offset_list.clear();
    BUFFER_SIZE = 1024;
    //doubleTo16bit = 32768.0;
    doubleTo16bit = 16384;

    mic_frame_id_gt = "base_footprint_gazebo";
    useGT = true;

    //load music sound file
    const std::string directory = ros::package::getPath("sound_simulation_robot")+"/data/";
    const std::string filename = "music1.wav";
    std::string file_path = directory+filename;
    load_soundfiles(file_path, music1);

    //load speech sound file
    const std::string filename2 ="german_voice.wav";
    std::string file_path2 = directory+filename2;
    load_soundfiles(file_path2, speech1);

    //load speech sound file
    const std::string filename3 ="speech_eng_jap.wav";
    std::string file_path3 = directory+filename3;
    load_soundfiles(file_path3, speech2);

    this->getParam("/microphone_config/sample_rate",sample_rate);
    this->getParam("/microphone_config/mic_count",num_mics);
    this->getParam("/microphone_config/audio_topic",audio_topic);
    mic_positions.clear();

    std::vector<double> position;
    geometry_msgs::Point p;

    for(int i=0;i<num_mics;i++){
        this->getParam("/microphone_config/mic"+std::to_string(i),position);

        p.x=position[0];
        p.y=position[1];
        p.z=position[2];

        mic_positions.push_back(p);
    }
//    for(auto i = mic_positions.begin(); i!= mic_positions.end(); i++){

//        std::cout << *i << std::endl;

//    }

    init_noise_buffer();

    publishSame = false;
    publishedFirst = false;

    use3d = true;

    mic_scale.clear();
    mic_scale.push_back(1);
    mic_scale.push_back(1);
    mic_scale.push_back(1);
    mic_scale.push_back(1);
    mic_scale.push_back(1);
    mic_scale.push_back(1);
    mic_scale.push_back(1);
    mic_scale.push_back(1);


}


void SoundSimulationRobot::set_config(sound_simulation_robot::SimulationConfig &config,uint32_t level){

    publishSame=config.publishSame;

    //reset
    publishedFirst = false;

    mic_scale.clear();
    mic_scale.push_back(config.scale_mic_0);
    mic_scale.push_back(config.scale_mic_1);
    mic_scale.push_back(config.scale_mic_2);
    mic_scale.push_back(config.scale_mic_3);
    mic_scale.push_back(config.scale_mic_4);
    mic_scale.push_back(config.scale_mic_5);
    mic_scale.push_back(config.scale_mic_6);
    mic_scale.push_back(config.scale_mic_7);

}

double SoundSimulationRobot::dist(const geometry_msgs::Point &pos1, const geometry_msgs::Point &pos2){

    const double dx=pos1.x-pos2.x;
    const double dy=pos1.y-pos2.y;
    const double dz=pos1.z-pos2.z;
    return sqrt(dx*dx+dy*dy+dz*dz);

}

void SoundSimulationRobot::init_noise_buffer(){

    for(int i=0;i<WHITE_NOISE_LENGTH;i++){
        white_noise[i]=(short) (normal_rand(gen)*doubleTo16bit);
    }
}

//Create a dummy signal
short SoundSimulationRobot::signal(int sample, int type){

#define AMP 1.0
    const int period=(int) (sample_rate/1000.0);

    switch(type){

    case SINE_440:{
//        return (short) (AMP*sin(2.0*M_PI*440.0*sample/sample_rate)*doubleTo16bit);

        return sin440vector[sin440_number-1][sample %sin440vector[sin440_number-1].size()];
    }

    case SINE_1000:
        return sin1000vector[sin1000_number-1][sample %sin1000vector[sin1000_number-1].size()];

//        return (short) (AMP*sin(2.0*M_PI*1000.0*sample/sample_rate)*doubleTo16bit);

    case PULSE_25:
//        sample%=period;
//        return (short) (sample<period*0.25 ? doubleTo16bit:-doubleTo16bit);
        return pulse25vector[pulse25_number-1][sample %pulse25vector[pulse25_number-1].size()];


    case PULSE_50:
//        sample%=period;
//        return (short) (sample<period*0.5 ? doubleTo16bit:-doubleTo16bit);
        return pulse50vector[pulse50_number-1][sample %pulse50vector[pulse50_number-1].size()];

    case WHITE_NOISE:
//        return white_noise[sample % WHITE_NOISE_LENGTH];
        return white_noise[sample % WHITE_NOISE_LENGTH];

    case MUSIC1:

        return music1_vector[music1_number-1][sample % music1_vector[music1_number-1].size()];
    case SPEECH1:

        return speech1_vector[speech1_number-1][sample % speech1_vector[speech1_number-1].size()];

    case SPEECH2:

        return speech1_vector[speech1_number-1][sample % speech1_vector[speech1_number-1].size()];

    case SINE_440_1000:
//    default:
//        const double freq1=sin(2.0*M_PI*440.0*sample/sample_rate);
//        const double freq2=cos(2.0*M_PI*1000.0*sample/sample_rate);
//        return (short) (AMP*(freq1+freq2)/2.0*doubleTo16bit);
        return sin440_1000vector[sin440_1000_number-1][sample %sin440_1000vector[sin440_1000_number-1].size()];
    }

}

short SoundSimulationRobot::noise(double &gain){
    return (short) (normal_rand(gen)*gain*doubleTo16bit);
}

double SoundSimulationRobot::signal_strength(std::vector<short> &signal,int &mic){

    double power=0.0;
    double s;
    for(int i=mic;i<signal.size();i+=num_mics){
        s=(double) signal[i]/doubleTo16bit;
        power+=s*s;
    }
    return power/signal.size()*num_mics;
}

void SoundSimulationRobot::load_soundfiles(std::string &filename,std::vector<short> &array){

    array.clear();
    audioFile.load(filename);

    const int n = audioFile.getNumSamplesPerChannel();
    const int channel = 0;

    ROS_INFO("Bit depth: %d, dtb %f", audioFile.getBitDepth(), doubleTo16bit);

    for(int i = 0; i<n; i++){

        //ROS_INFO("value: %f, multi %d", audioFile.samples[channel][(i)],(short)audioFile.samples[channel][(i)]*doubleTo16bit);

        array.push_back(audioFile.samples[channel][(i)]*doubleTo16bit);
    }


}

void SoundSimulationRobot::transform_mic_position(std::vector<geometry_msgs::Point> &mic_trans){
    //get relative position of sources (map) to microphone array (robot)
    tf::StampedTransform transform;
    try{

        if(useGT){
            listener->lookupTransform( map_frame_id, mic_frame_id_gt,
                                       ros::Time(0), transform);

        }

        else{

            listener->lookupTransform( map_frame_id, mic_frame_id,
                                       ros::Time(0), transform);
        }

    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    mic_trans.clear();
    geometry_msgs::Point m;

    for(int i = 0; i< mic_positions.size(); i++){

        m = mic_positions[i];
//        ROS_INFO("Mic %d from (%f,%f,%f)", i,m.x,m.y,m.z);
        const tf::Vector3 p(m.x,m.y,m.z);
        const tf::Vector3 t = transform*p;

        m.x = t.x();
        m.y = t.y();
        m.z = t.z();

        //ROS_INFO("Mic %d to (%f,%f,%f)", i,m.x,m.y,m.z);
        mic_trans.push_back(m);

    }

}

void SoundSimulationRobot::publishAudio(){
    //todo reset publishedFirst if position has changed
    if(sources.sound_poses.poses.size()>0){

//        //publish same message
//        if(publishSame && publishedFirst){
//            same_msg.frame_number = frame_num;
//            frame_num++;
//            pub_audio.publish(same_msg);

//        }

//        //generate new message
//        else{

            manyears::AudioStream msg;
            msg.frame_number=frame_num;
            std::vector<short> buffer(BUFFER_SIZE*num_mics,0);

            //todo loop over sources
            geometry_msgs::Point source;
            std::vector<geometry_msgs::Point> mic_trans;
            int type;
            int offset;
            double mic_scaling;
            double amplitude_scale;

            //just testing, remove;
            //mic_scale[0] = 2;

            //transfrom mic
            transform_mic_position(mic_trans);

                for(int i=0;i<BUFFER_SIZE;i++){
                    for(int j=0;j<mic_trans.size();j++){
                            short tempvariable = 0;
                            for(int m=0; m<sources.sound_poses.poses.size(); m++){
                                source = sources.sound_poses.poses[m].position;
                                type = sources.source_type[m];
                                offset = offset_list[m];
                                amplitude_scale = sources.amplitude_scale[m];
                                int delay=(int) (dist(mic_trans[j],source)/c*sample_rate);
//                                tempvariable += signal(sample-delay+offset,type);
                                  tempvariable += signal(sample,type);
                                if(type == MUSIC1){
                                    offset_list[m] = (offset + BUFFER_SIZE)% music1.size();
                                }
                                else if(type == SPEECH1){
                                    offset_list[m] = (offset + BUFFER_SIZE)% speech1.size();
                                }

                            }
                            mic_scaling = mic_scale[j];
                            buffer[i*num_mics+j]=tempvariable;
//                            ROS_INFO("The value of buffer is %d",buffer[i*num_mics+j]);

                     }
                     sample++;
//                        buffer[i*num_mics+j]+=amplitude_scale*mic_scaling*tempvariable;
                 }






//            //todo add for every source
//            //add noise
//            for(int j=0;j<num_mics;j++){
//                double sig=signal_strength(buffer,j);
//                double noise_gain=sqrt(sig/snr_linear);
//                for(int i=0;i<BUFFER_SIZE;i++){
//                    buffer[i*num_mics+j]+=noise(noise_gain);

//                }
//            }

            msg.stream_buffer=buffer;
            same_msg = msg;
            frame_num++;

            pub_audio.publish(msg);
//            if(publishSame){

//                publishedFirst = true;
//            }

    }


    else{
        std::vector<short> zero_signal(3000,0);
        manyears::AudioStream msg;
        msg.stream_buffer = zero_signal;
        pub_audio.publish(msg);

    }


}

void SoundSimulationRobot::get_update(const sound_msgs::GlobalSourceConstPtr msg){
    sources = *msg;
//    ROS_INFO("get update from marker........................................................................");
    sin440_number = 0;
    sin1000_number = 0;
    pulse25_number = 0;
    pulse50_number = 0;
    music1_number = 0;
    speech1_number = 0;
    speech2_number = 0;
    sin440_1000_number = 0;
    sin440vector.clear();
    sin1000vector.clear();
    sin440_1000vector.clear();
    pulse25vector.clear();
    pulse50vector.clear();
    music1_vector.clear();
    speech1_vector.clear();
    speech2_vector.clear();

        if(sources.sound_poses.poses.size()>0){
                geometry_msgs::Point sound_source;
                std::vector<geometry_msgs::Point> mic_trans;
                transform_mic_position(mic_trans);
                int type;

                for(int m=0; m<sources.sound_poses.poses.size(); m++){
                    sound_source = sources.sound_poses.poses[m].position;
                    type = sources.source_type[m];
                    Image Image1;
                    Image1.LoadParameter();
                    Image1.X_src << sound_source.x,sound_source.y,sound_source.z;
                    Image1.X_rcv << mic_trans[0].x,mic_trans[0].y,mic_trans[0].z;

                    //import beta
                    Image1.beta[0] = sources.wall1[m];
                    Image1.beta[1] = sources.wall2[m];
                    Image1.beta[2] = sources.wall3[m];
                    Image1.beta[3] = sources.wall4[m];
                    Image1.beta[4] = sources.celling[m];
                    Image1.beta[5] = sources.floor[m];

                    ROS_INFO("The position of sound_source %d are %f,%f,%f,",m,sound_source.x,sound_source.y,sound_source.z );
                    ROS_INFO("Microphone_Position are %f,%f,%f,",mic_trans[0].x,mic_trans[0].y,mic_trans[0].z );
                    //.............................................................................Runtime function
                        std::clock_t start;
                        double duration;

                        start = std::clock();

                        /* Your algorithm here */
                    //.............................................................................


                    Image1.MakeIMResp();
//                    //此处仍正确运行
//                    std::ofstream file("Image1.dat");
//                    if (file.is_open())
//                    {
//                                for(int i=0; i< Image1.FilterCoeffs.rows();i++){

//                                    file << Image1.FilterCoeffs(i,0) << std::endl;
//                        }

//                    }


                    switch(type){

                    case SINE_440:{
                        ROS_INFO("updating SINE_440");

                        sin440vector.push_back(Signal_convolution(type,Image1.FilterCoeffsvector));
//                            std::ofstream file("sin440vector.dat");
//                            if (file.is_open())
//                            {
//                                for(int i=0; i< sin440vector[sin440_number].size();i++){

//                                    file << sin440vector[sin440_number][i] << std::endl;
//                                }

//                            }
                        sin440_number+=1;

                        break;
                    }

                    case SINE_1000:
                        ROS_INFO("updating SINE_1000");
                        sin1000vector.push_back(Signal_convolution(type,Image1.FilterCoeffsvector));
                        sin1000_number+=1;
                        break;

                    case SINE_440_1000:
                        ROS_INFO("updating SINE_440_1000");
                        sin440_1000vector.push_back(Signal_convolution(type,Image1.FilterCoeffsvector));
                        sin440_1000_number+=1;
                        break;

                    case PULSE_25:
                        ROS_INFO("updating PULSE_25");
                        pulse25vector.push_back(Signal_convolution(type,Image1.FilterCoeffsvector));
                        pulse25_number+=1;
                        break;

                    case PULSE_50:
                        ROS_INFO("updating PULSE_25");
                        pulse50vector.push_back(Signal_convolution(type,Image1.FilterCoeffsvector));
                        pulse50_number+=1;
                        break;

                    case WHITE_NOISE:
                        ROS_INFO("updating WHITE_NOISE");
                        break;

                    case MUSIC1:
                        ROS_INFO("updating MUSIC1");
                        music1_vector.push_back(Signal_convolution(type,Image1.FilterCoeffsvector));
                        music1_number+=1;
                        break;

                    case SPEECH1:
                        ROS_INFO("updating SPEECH1");
                        speech1_vector.push_back(Signal_convolution(type,Image1.FilterCoeffsvector));
                        speech1_number+=1;
                        break;

                    case SPEECH2:
                        ROS_INFO("updating SPEECH2");
                        speech2_vector.push_back(Signal_convolution(type,Image1.FilterCoeffsvector));
                        speech2_number+=1;
                        break;

                    }
                    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
                 //.............................................................................Runtime function
                     ROS_INFO("Run Time is: %f",duration);
                 //.............................................................................

                 }
//                std::ofstream out("out1.dat",std::ios::out);
//                ROS_INFO("exporting data");
//                for(auto i = 0; i < sin440.size();i++){
//                    double a = 0;
//                    for(auto j = 0; j < sin440_number; j++){
//                        a += sin440vector[j][i];
//                    }
//                    sin440[i] = a*0.1;

//                    out <<  sin440[i] << std::endl;

//                }



        }


    ROS_INFO("Jump out of switch(type)");
    //take last value
    //todo: do for all sources in publish methode
    if(sources.snr.size() >0){

        snr_linear=pow(10,sources.snr[sources.snr.size()-1]/10.0);
    }

    else{
        snr_linear=pow(10,10);
    }


    //generate offset list
    //todo get old values

    offset_list.clear();

    for(int i = 0; i<sources.source_type.size();i++){

            //get sure all files will not start at the same time
            if(sources.source_type[i] == MUSIC1 || sources.source_type[i] == SPEECH1){

                const float rand_ = ((float) rand()) / (float) RAND_MAX;
                const float range_ = 100;

                offset_list.push_back(std::floor((rand_*range_)*BUFFER_SIZE));
            }

            else{

                offset_list.push_back(0);
            }

    }

    //reset
    publishedFirst = false;

}

bool SoundSimulationRobot::printTimeDifferenceGT(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    geometry_msgs::Point s;
    std::vector<geometry_msgs::Point> mic_trans;
    double dif;
    double t;
    double tmax;

    geometry_msgs::Point m1;
    geometry_msgs::Point m2;

    //transfrom mic
    transform_mic_position(mic_trans);

    for(int m=0; m<sources.sound_poses.poses.size(); m++){

        s = sources.sound_poses.poses[m].position;

        ROS_INFO("Source %d",m);

        for(int i = 0; i<mic_trans.size(); i++){

            m1 = mic_trans[i];

            for(int j = i+1; j< mic_trans.size(); j++){


                m2 = mic_trans[j];

                //calculate difference

                if(use3d){
                    dif = std::sqrt((m1.x-s.x)*(m1.x-s.x)+(m1.y-s.y)*(m1.y-s.y)+(m1.z-s.z)*(m1.z-s.z))-std::sqrt((m2.x-s.x)*(m2.x-s.x)+(m2.y-s.y)*(m2.y-s.y)+(m2.z-s.z)*(m2.z-s.z));
                }

                else{

                    //dif = std::sqrt((m1.x-s.x)*(m1.x-s.x)+(m1.y-s.y)*(m1.y-s.y))-std::sqrt((m2.x-s.x)*(m2.x-s.x)+(m2.y-s.y)*(m2.y-s.y));
                    dif = std::sqrt((m1.x-s.x)*(m1.x-s.x)+(m1.y-s.y)*(m1.y-s.y)+(m1.z-s.z)*(m1.z-s.z))-std::sqrt((m2.x-s.x)*(m2.x-s.x)+(m2.y-s.y)*(m2.y-s.y)+(m2.z-s.z)*(m2.z-s.z));

                }


                t = 1.0/c*dif*1000;

                tmax= 1.0/c*dif*1000;
                ROS_INFO("[%d,%d]->%0.2f, a->%0.2f ",i,j,t,1.0*mic_scale[j]/mic_scale[i]);
            }
        }
    }

    //show microphone position
    for(int i = 0; i< mic_trans.size(); i++){

        m1 = mic_trans[i];
        ROS_INFO("Mic %d from (%f,%f,%f)", i,m1.x,m1.y,m1.z);

    }

    //show sound source position

    for(int m=0; m<sources.sound_poses.poses.size(); m++){

        s = sources.sound_poses.poses[m].position;

        ROS_INFO("Source %d from (%f,%f,%f)",m,s.x,s.y,s.z);
    }

    return true;
}
//the function to caculate image response and the resulting sound signal at microphone
std::vector<short> SoundSimulationRobot::Signal_convolution(int type, std::vector<double> TF){

    std::vector<short> signal;
    int period=(int) (sample_rate/1000.0);
    switch(type){

    case SINE_440:{

        ROS_INFO("sin440 convolution");
        for (int i=0; i<4000; i++){
                int t=i/2;
//                short s=std::sin(2.0*M_PI*440.0*t/sample_rate)*doubleTo16bit;//设定频率为440Hz
                short s=std::sin(2.0*M_PI*440.0*t/sample_rate)*doubleTo16bit;
                signal.push_back(s);
        }

        return convolution(signal, TF);
//        return signal;
        break;
    }

    case SINE_1000:
        ROS_INFO("sin1000vector convolution");
        for (int i=0; i<4000; i++)//4秒，产生更多数据
            {
            int t=i/2;
            double s=std::sin(2.0*M_PI*1000.0*t/sample_rate)*doubleTo16bit;//设定频率为1000Hz
            signal.push_back(s);
        }

        return convolution(signal, TF);
//        return signal;
        break;

    case SINE_440_1000:{

        ROS_INFO("sin440_1000 convolution");
        for (int i=0; i<4000; i++){
                int t=i/2;
                short s=std::sin(2.0*M_PI*440.0*t/sample_rate)*doubleTo16bit;//设定频率为440Hz
                short j=std::cos(2.0*M_PI*1000.0*t/sample_rate)*doubleTo16bit;//设定频率为440Hz
                signal.push_back((s+j)/2);
        }

        return convolution(signal, TF);
//        return signal;
        break;
    }

    case PULSE_25:
        ROS_INFO("PULSE_25 convolution");
        int t ;
        for (int i=0; i<4000; i++)

            {

                t = i;
                t%=period;
                signal.push_back(t<period*0.25 ? doubleTo16bit:-doubleTo16bit);
        }
//        pulse25 = signal;
        return convolution(signal, TF);
        break;

    case PULSE_50:
        ROS_INFO("PULSE_50 convolution");
        for (int i=0; i<4000; i++)
            {

                t = i;
                t%=period;
                signal.push_back(t<period*0.5 ? doubleTo16bit:-doubleTo16bit);
        }
        return convolution(signal, TF);
        break;

     case WHITE_NOISE:
         ROS_INFO("WHITE_NOISE convolution");
         for(int i = 0; i < WHITE_NOISE_LENGTH; i++){
             signal.push_back(white_noise[i]);
         }

//        return convolution(signal, TF);
        return signal;
        break;

    case MUSIC1:
        ROS_INFO("MUSIC1 convolution");
        for(int i = 0; i < music1.size(); i++){
            signal.push_back(music1[i]);
        }

        return convolution(signal, TF);
        break;

    case SPEECH1:
        ROS_INFO("SPEECH1 convolution");
        for(int i = 0; i < speech1.size(); i++){
            signal.push_back(speech1[i]);
        }

       return convolution(signal, TF);
       break;

    case SPEECH2:
        ROS_INFO("SPEECH1 convolution");
        for(int i = 0; i < speech2.size(); i++){
            signal.push_back(speech2[i]);
        }
        return convolution(signal, TF);

       break;



    }

}

std::vector<short> SoundSimulationRobot::convolution(std::vector<short> f, std::vector<double> g){
    int nf = f.size();
    int ng = g.size();
    int n = nf + ng -1;
    std::vector<short> out(n);
    std::vector<short> out1(nf-ng);
    for(auto i(0); i < n; ++i){
        int jmn = (i >= ng-1)? i-(ng-1) : 0;
        int jmx = (i < nf -1)? i        : nf-1;
        for(auto j(jmn); j <= jmx; ++j){
            out[i] += (f[j] * g[i-j]);
        }

    }
    out.resize(nf);
    for(int i = 0;i < out1.size();i++){
        out1[i] =out[i+ng];
    }

    return out1;

}
void SoundSimulationRobot::moveBaseStatusCallback(const move_base_msgs::MoveBaseActionResultConstPtr msg){
    goalReached = false;
    waitForGoal = true;
    geometry_msgs::Point sound_source;
    std::vector<geometry_msgs::Point> mic_trans;
    geometry_msgs::Point microphone_point;
    transform_mic_position(mic_trans);
    Image Image2;
    Image2.LoadParameter();
    int type;
    if(waitForGoal){

        if(msg->status.status == 3){
            gotNewSource = true;
            ROS_INFO("New navigation goal was reached");

            goalReached = true;
            waitForGoal = false;
            if(sources.sound_poses.poses.size()>0){



                //clear all the existing signals

                sin440vector.clear();
                sin440_number = 0;

                sin1000vector.clear();
                sin1000_number = 0;


                sin1000vector.clear();
                sin440_1000_number = 0;


                sin1000vector.clear();
                pulse25_number = 0;


                sin1000vector.clear();
                pulse50_number = 0;


                sin1000vector.clear();
                music1_number = 0;


                speech1_vector.clear();
                speech1_number = 0;


                speech2_vector.clear();
                speech2_number = 0;

                for(int m=0; m<sources.sound_poses.poses.size(); m++){
                    ROS_INFO("calculating TF for new ROBOT position");
                    sound_source = sources.sound_poses.poses[m].position;
                    type = sources.source_type[m];
                    Image2.X_src << sound_source.x,sound_source.y,sound_source.z;
                    Image2.X_rcv << mic_trans[0].x,mic_trans[0].y,mic_trans[0].z;
                    ROS_INFO("sound_source are %f,%f,%f,",sound_source.x,sound_source.y,sound_source.z );
                    ROS_INFO("Microphone_Position are %f,%f,%f,",mic_trans[0].x,mic_trans[0].y,mic_trans[0].z );
                    //.............................................................................Runtime function
                        std::clock_t start;
                        double duration;

                        start = std::clock();

                        /* Your algorithm here */
                    //.............................................................................

                    Image2.MakeIMResp();
                    switch(type){

                    case SINE_440:

                      sin440vector.push_back(Signal_convolution(type,Image2.FilterCoeffsvector));
                      sin440_number+=1;
                      ROS_INFO("The size of sin440vector is %ld",sin440vector.size());
//                      microphone_point = mic_trans[0];
                        break;

                    case SINE_1000:

                      sin1000vector.push_back(Signal_convolution(type,Image2.FilterCoeffsvector));
                      sin1000_number+=1;
                        break;

                    case SINE_440_1000:

                      sin440_1000vector.push_back(Signal_convolution(type,Image2.FilterCoeffsvector));
                      sin440_1000_number+=1;
                        break;

                    case PULSE_25:

                        pulse25vector.push_back(Signal_convolution(type,Image2.FilterCoeffsvector));
                        pulse25_number+=1;
                        break;

                    case PULSE_50:

                        pulse50vector.push_back(Signal_convolution(type,Image2.FilterCoeffsvector));
                        pulse50_number+=1;
                        break;

                    case WHITE_NOISE:

//                      white_noise_vector =  Signal_convolution(type,Image2.FilterCoeffsvector);
                        break;

                    case MUSIC1:

                        music1_vector.push_back(Signal_convolution(type,Image2.FilterCoeffsvector));
                        music1_number+=1;
                        break;

                    case SPEECH1:

                        speech1_vector.push_back(Signal_convolution(type,Image2.FilterCoeffsvector));
                        speech1_number+=1;
                        break;

                    case SPEECH2:

                        speech2_vector.push_back(Signal_convolution(type,Image2.FilterCoeffsvector));
                        speech2_number+=1;
                        break;


                    }
                    // updating signal at  microphone
//                    publishAudio();
                    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
                 //.............................................................................Runtime function
                     ROS_INFO("Run Time is: %f",duration);
                 //.............................................................................

                 }

            }
        }

        else{
            ROS_INFO("Goal couldn't be reached");
        }
    }

}


int main(int argc,char **args){


    ros::init(argc,args,"sound_simulation");

    SoundSimulationRobot sim_obj = SoundSimulationRobot();
    ros::Rate loop_rate(sim_obj.sample_rate/sim_obj.BUFFER_SIZE);
    while(ros::ok()){

        sim_obj.publishAudio();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
