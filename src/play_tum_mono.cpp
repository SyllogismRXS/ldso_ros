#include "ros/ros.h"
#include "frontend/FullSystem.h"
#include "DatasetReader.h"

#include <memory>

int main(int argc, char **argv) {
    // Init ROS
    ros::init(argc, argv, "ldso");

    // Get ROS parameters
    ros::NodeHandle private_nh("~");

    std::string calibration_data_path;
    if (not private_nh.getParam("calibration_data_path", calibration_data_path)) {
        ROS_ERROR_STREAM("Missing calibration_data_path");
        return -1;
    }
    std::string vignette = calibration_data_path + "/vignette.png";
    std::string gammaCalib = calibration_data_path + "/pcalib.txt";
    std::string source = calibration_data_path + "/images.zip";
    std::string calib = calibration_data_path + "/camera.txt";

    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    // constants
    int startIdx = 0;
    int endIdx = 10000;
    float playbackSpeed = 0.5;    // 0 for linearize (play as fast as possible, while sequentializing tracking & mapping). otherwise, factor on timestamps.

    int lstart = startIdx;
    int lend = endIdx;
    int linc = 1;

    std::shared_ptr<ImageFolderReader> reader = (
        std::make_shared<ImageFolderReader>(ImageFolderReader::TUM_MONO, source, calib, gammaCalib, vignette));
    reader->setGlobalCalibration();

    std::vector<int> idsToPlay;
    std::vector<double> timesToPlayAt;
    for (int i = lstart; i >= 0 && i < reader->getNumImages() && linc * i < linc * lend; i += linc) {
        idsToPlay.push_back(i);
        if (timesToPlayAt.size() == 0) {
            timesToPlayAt.push_back((double) 0);
        } else {
            double tsThis = reader->getTimestamp(idsToPlay[idsToPlay.size() - 1]);
            double tsPrev = reader->getTimestamp(idsToPlay[idsToPlay.size() - 2]);
            timesToPlayAt.push_back(timesToPlayAt.back() + fabs(tsThis - tsPrev) / playbackSpeed);
        }
    }

    struct timeval tv_start;
    gettimeofday(&tv_start, NULL);
    clock_t started = clock();
    double sInitializerOffset = timesToPlayAt[0];

    for (int ii = 0; ii < (int) idsToPlay.size(); ii++) {
        int i = idsToPlay[ii];

        ImageAndExposure *img;
        img = reader->getImage(i);

        bool skipFrame = false;
        if (playbackSpeed != 0) {
            struct timeval tv_now;
            gettimeofday(&tv_now, NULL);
            double sSinceStart = sInitializerOffset + ((tv_now.tv_sec - tv_start.tv_sec) +
                                                       (tv_now.tv_usec - tv_start.tv_usec) / (1000.0f * 1000.0f));

            if (sSinceStart < timesToPlayAt[ii]) {
                // usleep((int) ((timesToPlayAt[ii] - sSinceStart) * 1000 * 1000));
            }
            else if (sSinceStart > timesToPlayAt[ii] + 0.5 + 0.1 * (ii % 2)) {
                printf("SKIPFRAME %d (play at %f, now it is %f)!\n", ii, timesToPlayAt[ii], sSinceStart);
                skipFrame = true;
            }
        }
        if (!skipFrame) {
            ROS_INFO("Frame");
            // publish here
            //fullSystem->addActiveFrame(img, i);
        }
        delete img;

        if (not ros::ok()) {
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
