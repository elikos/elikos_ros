//
// Created by ta11e4rand on 03/02/16.
//


#include "OpticalFlowMain.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "BlobDetection/RobotDetection.h"
#include "BlobDetection/OpticalFlowTracking.h"

using namespace cv;
using namespace std;

#define VIDEO_FILE "videos/cam_bas.mp4"
//#define VIDEO_FILE "videos/cam_bas_raccourci.mp4"
//#define VIDEO_FILE "videos/unRobot.mp4"
//#define VIDEO_FILE "videos/Calibration_Proche_Vert.mp4"
//#define VIDEO_FILE "videos/Calibration_Proche_Rouge.mp4"

int opticalFlowMain()
{
    // Ouvrir un video
    VideoCapture video(VIDEO_FILE);
    // Verifier que le video est bien ouvert
    if(!video.isOpened())
        return 2;

    // Declarer une matrice qui contiendra les images finales de couleur
    Mat threshold_w;
    Mat threshold_r;
    Mat threshold_g;
    // Declarer une matrice qui contiendra l'image finale pour les robots
    Mat robotsMat;

    //Indicate the number of frames between each calculation of cv2.goodFeaturesToTrack
    int featuresUpdateRate = 5;
    // Creer une fenetre nommee Output pour l'affichage
    namedWindow( "Input", WINDOW_AUTOSIZE );
    //namedWindow("White Threshold", WINDOW_AUTOSIZE );
    //namedWindow("Green Threshold", WINDOW_AUTOSIZE );
    //namedWindow("Red Threshold", WINDOW_AUTOSIZE );
    namedWindow("Robot detection", WINDOW_AUTOSIZE );

    moveWindow("Input", 0, 0);
    //moveWindow("White Threshold", 710, 0);
    //moveWindow("Green Threshold", 0, 580);
    //moveWindow("Red Threshold", 710, 580);
    moveWindow("Robot detection", 710, 580);

    //detection.createTrackbars();
    OpticalFlowTracking tracking;

    // Obtenir une première image de la camera et faire une première détection
    Mat frame;
    video >> frame;
    //Initialisation
    tracking.detectColor(frame, threshold_w, threshold_r, threshold_g, robotsMat);
    vector<cv::Point2f> p;
    cvtColor(frame,frame, COLOR_BGR2GRAY);
    goodFeaturesToTrack(frame, p,30, 0.01, 30);
    tracking.setPrevPoints(p);
    if(waitKey(30) >= 0) return -1;//normalement 30

    while(1) {
        // Obtenir une nouvelle image de la camera
        video >> frame;

        // Recommencer si le vidéo est terminé
        if (frame.empty()) {
            video = VideoCapture(VIDEO_FILE);
            video >> frame;
        }

        // Traiter l'image
        tracking.track(frame, threshold_w, threshold_r, threshold_g, robotsMat);
        // Afficher l'image
        imshow("Input", frame);
        //imshow("White Threshold", threshold_w);
        //imshow("Red Threshold", threshold_r);
        //imshow("Green Threshold", threshold_g);
        imshow("Robot detection", robotsMat);

        if(tracking.getTrackCounter()%featuresUpdateRate==0){
            vector<cv::Point2f> p;
            cvtColor(frame,frame, COLOR_BGR2GRAY);
            goodFeaturesToTrack(frame, p,30, 0.01, 30);
            tracking.setPrevPoints(p);
        }
        // Attendre une touche pendant 30ms avant de continuer de la boucle
        if(waitKey(5) >= 0) break;//normalement 30
    }
    // Destroy the windows we have created
    cvDestroyWindow("Input");
    cvDestroyWindow("Robot detection");
    //cvDestroyWindow("White Threshold");
    //cvDestroyWindow("Red Threshold");
    //cvDestroyWindow("Green Threshold");
    return 0;
}