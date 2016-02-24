#include "circleDetectionTestMain.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "EdgeDetection/EdgeDetection.h"

using namespace cv;
using namespace std;

#define VIDEO_FILE "videos/cam_bas.mp4"
#define VIDEO_FILE2 "videos/unRobot.mp4"

int circleDetectionTestMain()
{
    // Ouvrir un video
    VideoCapture video(VIDEO_FILE);

    // Verifier que le video est bien ouvert
    if(!video.isOpened())
        return -1;

    // Declarer une matrice qui contiendra l'image finale
    Mat circlesMat;

    // Creer une fenetre nommee Output pour l'affichage
    namedWindow( "Input", WINDOW_AUTOSIZE );
    moveWindow("Input", 0, 0);

    EdgeDetection edgeDetection;

    while(1) {
        // Obtenir une nouvelle image de la camera
        Mat frame;
        video >> frame;

        // Traiter l'image
        edgeDetection.trackCircles(frame, circlesMat);

        // Afficher l'image
        imshow("Input", frame);
        imshow("Circle detection", circlesMat);

        // Attendre une touche pendant 30ms avant de continuer de la boucle
        if(waitKey(30) >= 0) break;//normalement 30
    }
    // Destroy the windows we have created
    cvDestroyWindow("Input");
    cvDestroyWindow("Circle detection");

    return 0;
}