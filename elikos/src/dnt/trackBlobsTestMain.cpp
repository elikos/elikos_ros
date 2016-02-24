#include "trackBlobsTestMain.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include "EdgeDetection/EdgeDetection.h"
#include "BlobDetection/BlobDetection.h"

using namespace cv;
using namespace std;

#define VIDEO_FILE "videos/cam_bas.mp4"
//#define VIDEO_FILE "videos/cam_bas_raccourci.mp4"
//#define VIDEO_FILE "videos/unRobot.mp4"
//#define VIDEO_FILE "videos/Calibration_Proche_Vert.mp4"
//#define VIDEO_FILE "videos/Calibration_Proche_Rouge.mp4"
//#define VIDEO_FILE "videos/Calibration_Montage.mp4"

int trackBlobsTestMain()
{
    // Ouvrir un video
    VideoCapture video(VIDEO_FILE);

    // Verifier que le video est bien ouvert
    if(!video.isOpened())
        return -1;

    // Declarer une matrice qui contiendra l'image finale
    Mat threshold_w;
    Mat threshold_r;
    Mat threshold_g;

    // Creer une fenetre nommee Output pour l'affichage
    namedWindow( "Input", WINDOW_AUTOSIZE );
    //namedWindow("White Threshold", WINDOW_AUTOSIZE );
    namedWindow("Green Threshold", WINDOW_AUTOSIZE );
    namedWindow("Red Threshold", WINDOW_AUTOSIZE );

    moveWindow("Input", 0, 0);
    //moveWindow("White Threshold", 710, 0);
    moveWindow("Green Threshold", 0, 580);
    moveWindow("Red Threshold", 710, 580);

    BlobDetection detection;
    detection.createTrackbars();

    while(1) {
        // Obtenir une nouvelle image de la camera
        Mat frame;
        video >> frame;
        //frame = imread("videos/Image1.png", CV_LOAD_IMAGE_COLOR);

        // Recommencer si le vidéo est terminé
        if (frame.empty()) {
            video = VideoCapture(VIDEO_FILE);
            video >> frame;
        }

        // Traiter l'image
        detection.trackBlobs(frame, threshold_w, threshold_r, threshold_g);

        // Afficher l'image
        imshow("Input", frame);
        //imshow("White Threshold", threshold_w);
        imshow("Red Threshold", threshold_r);
        imshow("Green Threshold", threshold_g);

        // Attendre une touche pendant 30ms avant de continuer de la boucle
        if(waitKey(30) >= 0) break;//normalement 30
    }
    return 0;
}
