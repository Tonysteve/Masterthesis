#include "Image_source.h"
#include "AudioFile.h"
# define M_PI           3.14159265358979323846  /* pi */
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <complex>
#include <fstream>


std::complex<double> i(0, 1);

void Image::LoadParameter(){

//    X_src << 1, 1, 1;
    scale = 1/(4 *M_PI);
    room << 8, 4.8, 3;
    beta = beta.Ones(6, 1) * 0.8;
    LimAtten_dB = 45;
    c = 343.4;
    Fs = 8000;
    X_rcv << (1.85), (1.87), (1.2);
    X_src << (1.85), (1.87), (1.5);
    Atmospheric_Absorption();
//    sin440.resize(12000,0);

}

double Image::Atmospheric_Absorption()
{
    T_0 = 293.15;
    T= 293.15;
    P_s = 101.325;
    P_0 = P_s;
    H = 4.7*pow(10,-3);
    f_r_N = 9+280*H;
    f_r_O = 24 +4.04*pow(10,4)*H*(0.02+H)/(0.391+H);


    source_frequency = 2000;
    distance =2;



    alpha= pow(source_frequency,2)*(1.84*pow(10,-11)+(0.10680*exp(-3352/T)*f_r_N)/(pow(source_frequency,2)+pow(f_r_N,2))+
         0.01278*exp(-2239.1/T)*f_r_O/(pow(source_frequency,2)+pow(f_r_O,2)));
    std::cout << "alpha is" << alpha << std::endl;
    A_alpha = exp(-alpha*distance/2);
    std::cout << "A_alpha is" << A_alpha << std::endl;

}

Eigen::VectorXd Image::sinc(Eigen::VectorXd &timepoints){
    for(int i=0; i< timepoints.size(); i++){
        double a = timepoints(i);
        if(a==0){
            timepoints(i)=1;
        }
        else{
            timepoints(i)= (sin(M_PI*a)/(M_PI*a));
        }
    }
    return timepoints;
}


int Image::Check_nDim(double a, double b, double d, double l, double m)
{
    int FoundNValBelowLim = 0;
    int n = 1;
   //create a temporary Eigen::vector TimePoints1
    Eigen::VectorXd TimePoints1;
    Eigen::Vector3d abc(2 * a - 1, 2 * b - 1, 2 * d - 1);
    Eigen::Vector3d nlm(n, l, m);
    Eigen::Vector3d dist_vector = abc.cwiseProduct(X_src) + X_rcv - Rr.cwiseProduct(nlm);
    double dist = dist_vector.norm();
    double foo_time = dist / c;
    while (foo_time <= MaxDelay)   //if delay is below TF length limit for n=1, check n=2,3,4...
    {

        Eigen::Matrix<double, 6, 1> image_sources;
        image_sources << n - a, n, l - b, l, m - d, m;
        Eigen::Matrix<double, 6, 1> foo_amplitude1 = beta.array().pow(image_sources.array().abs());
        double foo_amplitude;
            foo_amplitude = foo_amplitude1.prod()*scale* A_alpha / ( dist);


        TimePoints1 = (TimePoints.array()-foo_time)*Fs;
        TimePoints1 = sinc(TimePoints1).array()*foo_amplitude;
        SpectrumVec += TimePoints1;
        n += 1;
        nlm << n, l, m;
        dist_vector = abc.cwiseProduct(X_src) + X_rcv - Rr.cwiseProduct(nlm);
        dist = dist_vector.norm();
        foo_time = dist / c;

    }
    if (n != 1)
    {
        FoundNValBelowLim = 1;
    }
    n = 0;
    nlm << n, l, m;
    dist_vector = abc.cwiseProduct(X_src) + X_rcv - Rr.cwiseProduct(nlm);
    dist = dist_vector.norm();
    foo_time = dist / c;

    while (foo_time <= MaxDelay)   //if delay is below TF length limit for n=1, check n=2,3,4...
    {

        Eigen::Matrix<double, 6, 1> image_sources;
        image_sources << n - a, n, l - b, l, m - d, m;
        Eigen::Matrix<double, 6, 1> foo_amplitude1 = beta.array().pow(image_sources.array().abs());
        double foo_amplitude;
                    foo_amplitude = foo_amplitude1.prod()*scale * A_alpha / ( dist);


        TimePoints1 = (TimePoints.array()-foo_time)*Fs;
        TimePoints1 = sinc(TimePoints1).array()*foo_amplitude;
        SpectrumVec += TimePoints1;

        n -= 1;
        nlm << n, l, m;
        dist_vector = abc.cwiseProduct(X_src) + X_rcv - Rr.cwiseProduct(nlm);
        dist = dist_vector.norm();
        foo_time = dist / c;
    }
    if (n != 0)
    {
        FoundNValBelowLim = 1;
    }
    return FoundNValBelowLim;
}

int Image::Check_lDim(double a, double b, double d, double m)
{

   int FoundLValBelowLim = 0;
   int l = 1;      // Check delay values for l = 1 and above
   int FoundNValBelowLim = Check_nDim(a,b,d,l,m);
   while (FoundNValBelowLim == 1)
   {
       l +=1,
       FoundNValBelowLim = Check_nDim(a, b, d, l, m);
   }
   if (l != 1)
   {
       FoundLValBelowLim = 1;
   }
   l = 0;
   FoundNValBelowLim = Check_nDim(a, b, d, l, m);
   while (FoundNValBelowLim == 1)
   {
       l -= 1,
           FoundNValBelowLim = Check_nDim(a, b, d, l, m);
   }
   if (l != 0)
   {
       FoundLValBelowLim = 1;
   }
   return FoundLValBelowLim;
}

void Image::MakeIMResp()
// This function returns the time coefficients of the filter (transfer
// function) in 'h' and its corresponding frequency response 'H'.The filter
// coefficients are real and non - normalised.The first value in vector 'h',
// h(1), corresponds to time t = 0. The number of coefficients returned is
// variable and results from the value of 'LimAtten' defined by the user :
// the filter length will be as large as necessary to capture all the
// relevant highest - order reflections.
{



    double V, aa_sab, T60val, foo;
    int TForder;

    if (X_rcv[0] >= room[0] | X_rcv[1] >= room[1] | X_rcv[2] >= room[2] | X_rcv[0] <= 0 | X_rcv[1] <= 0 | X_rcv[2] <= 0 )
    {
        throw "Receiver must be within the room boundaries.";
    }
    else if(X_src[0] >= room[0] | X_src[1] >= room[1] | X_src[2] >= room[2] | X_src[0] <= 0 | X_src[1] <= 0 | X_src[2] <= 0 )
    {
        throw "Source must be within the room boundaries.";
    }
    for (int i = 0; i < 6; i++)
    {
        if (beta[i] >= 1 | beta[i] < 0)
        {
            throw "Parameter ''beta'' must be in range [0...1]";
        }
    }
    for (int i = 0; i < 3; i++)
    {
             Rr[i] = 2 * room[i];  //Room dimensions
    }
    Eigen::Vector3d X_rcv_X_src = X_rcv - X_src;
    double DPdel = X_rcv_X_src.norm() / c;                          //direct path delay in [s]
    if (beta[0] != 0 && beta[1] != 0 && beta[2] != 0 && beta[3] != 0 && beta[4] != 0 && beta[5] != 0) {

        if (measT60 != 0)
        {

             V = room[0] * room[1] * room[2];
             aa_sab = (2 - beta[0] * beta[0] - beta[1] * beta[1]) * room[1] * room[2]
                 + (2 - beta[2] * beta[2] - beta[3] * beta[3]) * room[0] * room[2]
                 + (2 - beta[4] * beta[4] - beta[5] * beta[5]) * room[0] * room[1];
             T60val = 0.161 * V / aa_sab;              //Sabine's reverberation time in [s]
        }
        else
        {
            T60val = measT60;                         //practical T60 measurement determines real energy decay in TF!
        }
        foo = LimAtten_dB * T60val / 60;              //desired length of TF (TF decays by 60dB for T60 seconds after direct path delay)

        MaxDelay = DPdel + foo;                       //maximum delay in TF: direct path plus TForder
    }
    else
    {
        MaxDelay = 2 * DPdel;                         //anechoic case: allow for 2 times direct path in TF
    }
    TForder = ceil(MaxDelay * Fs);                    //total TF length

    //initialize TimePoints
    TimePoints.resize(TForder);
    for(int i = 0; i < TForder; i++){
        double a = (float)i/Fs;
        TimePoints(i) = a;
//        std::cout << TimePoints(i) << std::endl;
    }


    SpectrumVec = SpectrumVec.setZero(TForder,1);
    if (silentflag)
    {
        std::cout << "[MakeIMResp] Computing transfer function" << std::endl;

    }
     //----summation over room dimensions :
    for (int a = 0; a < 2; a++)
    {
        for (int b = 0; b < 2; b++)
        {
            for (int d = 0; d < 2; d++)
            {
                if (!silentflag)
                {
                    std::cout << "." << std::endl;
                }
                int m = 1;                            //Check delay values for m=1 and above
                int FoundLValBelowLim = Check_lDim(a, b, d, m);
                while(FoundLValBelowLim == 1)
                {
                    m += 1;
                    FoundLValBelowLim = Check_lDim(a, b, d, m);
                }
                m = 0;                                //Check delay values for m=0 and below
                FoundLValBelowLim = Check_lDim(a, b, d, m);
                while (FoundLValBelowLim == 1)
                {
                    m -= 1;
                    FoundLValBelowLim = Check_lDim(a, b, d, m);
                }
            }
        }

    }


    if (!silentflag)
    {
        std::cout << "\n" << std::endl;
    }
//    SpectrumVec(0) = SpectrumVec(0)/2.0;
    FilterCoeffsvector.clear();
    for(int i = 0;i < SpectrumVec.size(); i++){
        FilterCoeffsvector.push_back(SpectrumVec(i));
    }



}

Eigen::MatrixXd Image::readCSV(std::string file,  int cols) {

    std::ifstream in(file);

    std::string line;

    int row = 0;
    int col = 0;

    Eigen::MatrixXd res = Eigen::MatrixXd(8000, cols);

    if (in.is_open()) {

        while (getline(in, line)) {

            char* ptr = (char*)line.c_str();
            int len = line.length();

            col = 0;

            char* start = ptr;
            for (int i = 0; i < len; i++) {

                if (ptr[i] == ',') {
                    res(row, col++) = atof(start);
                    start = ptr + i + 1;
                }
            }
            res(row, col) = atof(start);

            row++;
        }

        in.close();
    }
    res.conservativeResize(row, col+1);
    return res;
}

void Image::convolution(Eigen::MatrixXd TF, std::string WAV)
{
    AudioFile<double> audiofile;
    audiofile.load(WAV);
    Eigen::MatrixXd SrcSignal(audiofile.getNumSamplesPerChannel(), 1);
    for (int i = 0; i < audiofile.getNumSamplesPerChannel(); i++)
    {
        SrcSignal(i,0) =  audiofile.samples[0][i] ;
    }

    int Tflen = TF.rows();
    int SrcSignallen = SrcSignal.rows();
    Eigen::MatrixXd AuData = AuData.Zero(SrcSignal.rows()+TF.rows()-1, TF.cols());
    //std::cout << AuData.rows() << AuData.cols() << std::endl;
    //std::cout << TF.rows() << TF.cols() << std::endl;
    for (int i = 0; i < TF.cols(); i++)
    {
        for (int j = 0; j < AuData.rows(); j++)
        {
            for (int k = std::max(0, j + 1 - Tflen); k <= std::min(j, SrcSignallen - 1); k++)
            {
                AuData(j, i) += SrcSignal(k, i) * TF(j - k,i);
            }
        }
    }

    for (int i = 0; i < audiofile.getNumSamplesPerChannel(); i++)
    {
        audiofile.samples[0][i] = AuData(i, 0) ;
    }
    audiofile.save("Simualtion1mbeta0_8.wav", AudioFileFormat::Wave);

}

void Image::convolution_sin(Eigen::MatrixXd TF)
{
    double t,s;
    Eigen::VectorXd SrcSignal;
    SrcSignal.setZero(4000);
     for (int i=0; i<SrcSignal.rows(); i++)//4秒，产生更多数据
         {
//         double t=i / 20;

//             double s=std::sin(2.0*M_PI*440.0*t/44100)*16384;
             double s=std::sin(2.0*M_PI*i/400)*10000;
             SrcSignal(i) =s;
         }



    int Tflen = TF.rows();
//    for (int i=0; i<TF.rows(); i++)//4秒，产生更多数据
//    {
////            file << TF(i,0) << std::endl;
//        std::cout << TF(i,0) << std::endl;

//    }
    int SrcSignallen = SrcSignal.rows();
    std::vector<double> AuData(SrcSignal.rows()+TF.rows()-1,0);
    {
       for (int j = 0; j < AuData.size(); j++)

        {
            for (int k = std::max(0, j + 1 - Tflen); k <= std::min(j, SrcSignallen - 1); k++)
            {
                  AuData[j] += SrcSignal(k, 0) * TF(j - k,0);
            }
        }
    }
    AuData.resize(SrcSignal.rows());
    sin440.resize(SrcSignal.rows()-Tflen);
        std::ofstream file("sin.csv");
        if (file.is_open())
        {
            for (int i=0; i<sin440.size(); i++)//4秒，产生更多数据
            {
                    sin440[i] += AuData[i+Tflen];
                    file << sin440[i] << std::endl;

            }



        }





}
