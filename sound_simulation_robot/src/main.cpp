#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include "Image_source.h";

using namespace std;
using namespace Eigen;


void main()
{
    Image Image1;

    Image1.X_src << 1, 1, 1;
    Image1.room << 3, 4, 2.5;
    Image1.beta = Image1.beta.Ones(6, 1) * 0.8;
    Image1.LimAtten_dB = 45;
    Image1.c = 343.4;
    Image1.Fs = 16000;
    MatrixXd X_rcv(8, 3);
    X_rcv << 1.6, 1, 1.5,
             1.4, 1, 1.5,
             1.5, 1, 1.6,
             1.4, 1, 1.6,
             1.3, 1, 1.7,
             1.2, 1, 1.7,
             1.1, 1, 1.8,
             1.1, 1, 1.8;
    Image1.X_rcv << 1.6, 1, 1.5;
    Image1.MakeIMResp();
    //MatrixXd FilterCoeffs1(Image1.FilterCoeffs.rows(),X_rcv.rows());
    //cout << FilterCoeffs1.rows() << endl << FilterCoeffs1.cols() << endl;
    //FilterCoeffs1.col(0).array() = Image1.FilterCoeffs.col(0).array();
  /*  for (int i = 1; i < X_rcv.rows(); i++)
    {
        Image1.X_rcv.array() = X_rcv.row(i).array();
        Image1.MakeIMResp();
        cout << Image1.FilterCoeffs.rows() << endl << Image1.FilterCoeffs.cols() << endl;*/
    //    FilterCoeffs1.col(i).array() = Image1.FilterCoeffs.col(0).array();
    //    
    //}
    //Image1.X_rcv << 1.6, 1, 1.5;
    //Image1.MakeIMResp();
    MatrixXd FilterCoeffs1;
    FilterCoeffs1 = Image1.FilterCoeffs;
    cout <<"row number is" << FilterCoeffs1.rows() << endl 
        <<"column number is"<< FilterCoeffs1.cols() << endl;
    ofstream file("test1.csv");
    if (file.is_open())
    {
        file << FilterCoeffs1 << endl;
    }
    //MatrixXd res = Image1.readCSV("test1.csv",1);
    //Image1.convolution(res, "speech.wav");
}