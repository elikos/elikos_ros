//
// Created by tonio on 21/03/16.
//

#include "LineIntersections.h"

std::vector<cv::Point> findCentroids(std::vector<int> clusters, std::vector<cv::Point> linePoints) {
    // Count number of clusters
    int nClusters = 0;
    for (auto clusterID : clusters) {
        if (clusterID + 1 > nClusters) {
            nClusters = clusterID + 1;
        }
    }

    std::vector<cv::Point> centroids(nClusters);
    std::vector<int> nLines(nClusters, 0);

    for (int i = 0; i < nClusters; i++) {
        for (int j = 0; j < linePoints.size(); ++j) {
            if (clusters[j] == i) {
                centroids[i] = cv::Point2i(centroids[i].x + linePoints[j].x, centroids[i].y + linePoints[j].y);
                nLines[i]++;
            }
        }
    }

    for (int i = 0; i < nClusters; i++) {
        if (nLines[i] == 0) continue;
        centroids[i] = cv::Point(centroids[i].x / nLines[i], centroids[i].y / nLines[i]);
    }

    return centroids;
}

std::vector<cv::Vec2f> getRhoTheta(std::vector<cv::Point> linePoints) {
    std::vector<cv::Vec2f> rhoTheta;
    if (linePoints.size() == 0) return rhoTheta;
    for (auto pointPair : linePoints) {
        float theta = atan2f(pointPair.y, pointPair.x);
        float rho = pointPair.x / cosf(theta);
        rhoTheta.push_back(cv::Vec2f(rho, theta));
    }

    return rhoTheta;
}

cv::Vec2f getRhoTheta(cv::Point linePoint) {
    float theta = atan2f(linePoint.y, linePoint.x);
    float rho = linePoint.x / cosf(theta);
    return cv::Vec2f(rho, theta);
}

cv::Point findIntersection(cv::Point linePoint1, cv::Point linePoint2) {
    auto rhoTheta1 = getRhoTheta(linePoint1);
    auto rhoTheta2 = getRhoTheta(linePoint2);

    float a1 = -2000 * cosf(rhoTheta1[1]);
    float a2 = -2000 * cosf(rhoTheta2[1]);
    float b1 = -2000 * sinf(rhoTheta1[1]);
    float b2 = -2000 * sinf(rhoTheta2[1]);
    float c1 = a1 * linePoint1.x + b1 * linePoint1.y;
    float c2 = a2 * linePoint2.x + b2 * linePoint2.y;
    float det = a1 * b2 - a2 * b1;
    float angle = fabs(rhoTheta1[1] - rhoTheta2[1]);

    if (fabs(det) < 10000.0 || angle < M_PI_4) {
        return cv::Point(0,0);
    } else {
        return cv::Point((int)roundf((b2 * c1 - b1 * c2) / det), (int)roundf((a1 * c2 - a2 * c1) / det));
    }
}

std::vector<cv::Point> findIntersections(std::vector<int> clusters, std::vector<cv::Point> linePoints) {
    std::vector<cv::Point> intersections;
    for (int i = 0; i < linePoints.size(); ++i) {
        for (int j = 0; j < linePoints.size(); ++j) {
            // Don't count intersection with self or twice with other line, skip outliers
            if (i <= j || clusters[i] == -1 || clusters[j] == -1) continue;
            cv::Point intersection = findIntersection(linePoints[i], linePoints[j]);

            // Store intersection if not null
            if (intersection != cv::Point(0, 0) &&
                (intersection.x > 0 && intersection.x < 640) &&
                (intersection.y > 0 && intersection.y < 480)){
                intersections.push_back(intersection);
            } else {
            }
        }
    }

    return intersections;
}

std::vector<cv::Point> findIntersections(std::vector<int> clusters_x, std::vector<int> clusters_y,
                                         std::vector<cv::Point> linePoints_x, std::vector<cv::Point> linePoints_y) {
    std::vector<cv::Point> intersections;
    for (int i = 0; i < linePoints_x.size(); ++i) {
        for (int j = 0; j < linePoints_y.size(); ++j) {
            // Skip outliers
            if (clusters_x[i] == -1 || clusters_y[j] == -1) continue;
            cv::Point intersection = findIntersection(linePoints_x[i], linePoints_y[j]);

            // Store intersection if not null
            if (intersection != cv::Point(0, 0) &&
                (intersection.x > 0 && intersection.x < 640) &&
                (intersection.y > 0 && intersection.y < 480)){
                intersections.push_back(intersection);
            } else {
            }
        }
    }

    return intersections;
}
