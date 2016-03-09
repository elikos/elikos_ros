#include "EdgeDetection.h"

EdgeDetection::EdgeDetection() {
    //initialisation des anciennes pos.
    /*for (int j = 0; j < 20; j++) {
        ancPos[j][0] = 0;
        ancPos[j][1] = 0;
    }*/
    //robots initialisation
    for (int j = 0; j < 20; j++) {
        robots.push_back(RobotDesc(j, 0, 0));
    }
}

void EdgeDetection::FindEdges(const cv::Mat &input, cv::Mat &output) {
    cvtColor(input, output, CV_BGR2GRAY);
    GaussianBlur(output, output, cv::Size(7,7), 1.5, 1.5);
    Canny(output, output, 0, 30, 3);
}


void EdgeDetection::trackCircles(const cv::Mat &input, cv::Mat &output) {//TODO:Placer cette méthode dans EdgeDetection

    //Check if the input is empty
    if(!input.data)
        cerr << "Input of trackCircles is empty";

    //convertion of the input
    cvtColor(input, output, CV_BGR2GRAY);

    //reduce the noise to avoid false circle detection
    GaussianBlur(output, output,Size(9,9),2,2);

    //container for circles
    vector<Vec3f> circles;

    //Apply the hough transform to find the circles
    //1st param: output Mat
    //2nd param: vector of circles found
    //3rd param: method, currently this is the only one that works
    //4th param: resolution ratio, 1 keeps the highest resolution possible
    //5th param: less it is, less the detection is accurate (suggestion 50 to 500), default 100
    //6th param: less it is, more we get false circle detection (suggestion 50 to 500), default 100
    //7th and 8th params: min and max circle radius , default 0 (don't care)
    HoughCircles(output, circles, CV_HOUGH_GRADIENT, 0.5, output.rows/8,200,50); //TODO: add trackbars to set the parameters



    //Draw the circles detected
    for(size_t i=0; i<circles.size();i++)
    {
        //index display
        char index[1];

        //sprintf(index, "%d", i);
        putText(output, index, cvPoint(circles[i][0],circles[i][1]), FONT_HERSHEY_PLAIN,3, Scalar(0,0,0),4);

        //circle center
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        //radius
        int radius = cvRound(circles[i][2]);
        //circle center
        circle(output, center, 3, Scalar(0,255,0), -1,8,0);
        //circle outline
        circle(output, center, radius, Scalar(0,0,255),3,8,0);


        //decompte des robots, fait passer l'algorithme en O(n²)
        //on vérifie si la position varie de plus de 1/20 de la taille de l'image, si oui on considère qu'il s'agit d'un nouveau robot.
       /* if(abs(ancPos[i][0]-circles[i][0])>64&&abs(ancPos[i][1]-circles[i][1])>48) {//64 et 48 représente 1/10
            nbRobotsTot++;
        }
        for(size_t j=0; j<circles.size();j++)//vérification s'il n'y a pas deux robots superposés
        {
            if(j!=i&&abs(ancPos[j][0]-circles[i][0])<64&&abs(ancPos[j][1]-circles[i][1])<48) {//64 et 48 représente 1/10
                nbRobotsTot--;
            }
        }
        //sauvegarde des anciennes positions
        ancPos[i][0]=circles[i][0];
        ancPos[i][1]=circles[i][1];*/


        if(abs(robots.at(i).getXPos()-circles[i][0])>64&&abs(robots.at(i).getYPos()-circles[i][1])>48) {//64 et 48 représente 1/10
            nbRobotsTot++;
        }
        for(size_t j=0; j<circles.size();j++)//vérification s'il n'y a pas deux robots superposés
        {
            if(j!=i&&abs(robots.at(j).getXPos()-circles[i][0])<64&&abs(robots.at(j).getYPos()-circles[i][1])<48) {//64 et 48 représente 1/10
                nbRobotsTot--;
            }
        }
        //sauvegarde des anciennes positions
        robots.at(i).setXPos(circles[i][0]);
        robots.at(i).setYPos(circles[i][1]);


    }
    //Affichage du décompte de robots
    char decompte[12];
//    sprintf(decompte, "nb visible:%d", circles.size());
    putText(output, decompte, cvPoint(0,FRAME_HEIGHT-80), FONT_HERSHEY_PLAIN,3, Scalar(0,0,0),4);
    char decompteTot[10];
    sprintf(decompteTot, "nb total:%d", nbRobotsTot);
    putText(output, decompteTot, cvPoint(0,FRAME_HEIGHT), FONT_HERSHEY_PLAIN,3, Scalar(0,0,0),4);
}

