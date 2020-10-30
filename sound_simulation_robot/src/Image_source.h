#include <Eigen/Dense>
#include <complex>
#include <iostream>


//using namespace Eigen;



class Image
{
public:
    double scale;
    double Fs;              // sampling frequency in Hz
    double c;               // propagation speed of acoustic waves in m/s
    Eigen::Vector3d room;         // room dimensions in m
    Eigen::MatrixXi micpos;          // [x y z] position of array microphones in m
    Eigen::Matrix<double, 6, 1> beta;         // reflection coefficients in range [0 ... 1]
    double straj;           // [x y z] positions in source trajectory in m
                            // straight line in front of mic pair, 50 source points
    Eigen::Vector3d X_src;        // [x y z] positions of source microphone
    Eigen::Vector3d X_rcv;        // [x y z] positions of receiver
    Eigen::Vector3d Rr;            //
    double measT60;         // scalar, measured reverberation time in seconds.If the T60 value has been practically measured(e.g. using IMRevTimeAnalysis.m) for the
                            //current setup, set 'measT60' to this value for a more accurate computation of the TF.Otherwise leave empty, i.e.define as[].
    double LimAtten_dB;
    double silentflag;
    double MaxDelay;
    Eigen::Matrix<double, Eigen::Dynamic, 1 > SpectrumVec;
    std::vector<double> FilterCoeffsvector;
    Eigen::VectorXd TimePoints;

    std::vector<double> sin440;
//..Parameter for Atmospheric_Absorption......................................................................
    double T_0;
    double T;
    double P_s;
    double P_0;
    double H;
    double f_r_N;
    double f_r_O;
    double alpha;
    double A_alpha;
    double distance;
    double source_frequency;


//........................................................................
    double Atmospheric_Absorption();
    void MakeIMResp();
    int Check_nDim(double, double, double, double, double);
    int Check_lDim(double, double, double, double);
    Eigen::MatrixXd readCSV(std::string, int);
    void convolution(Eigen::MatrixXd, std::string);
    void convolution_sin(Eigen::MatrixXd TF);
    void LoadParameter();
    Eigen::VectorXd sinc(Eigen::VectorXd &timepoints);
};
