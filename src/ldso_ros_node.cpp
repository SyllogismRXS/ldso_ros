#include "ros/ros.h"
#include "frontend/FullSystem.h"
#include "DatasetReader.h"

#include <memory>

int main(int argc, char **argv) {
    // Init ROS
    ros::init(argc, argv, "ldso");

    // Get ROS parameters
    ros::NodeHandle private_nh("~");

    std::string voc_path;
    if (not private_nh.getParam("voc_path", voc_path)) {
        ROS_ERROR_STREAM("Missing voc_path");
        return -1;
    }

    std::string calibration_data_path;
    if (not private_nh.getParam("calibration_data_path", calibration_data_path)) {
        ROS_ERROR_STREAM("Missing calibration_data_path");
        return -1;
    }
    std::string vignette = calibration_data_path + "/vignette.png";
    std::string gammaCalib = calibration_data_path + "/pcalib.txt";
    std::string source = calibration_data_path + "/images.zip";
    std::string calib = calibration_data_path + "/camera.txt";
    std::string output_file = "./results.txt";

    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    // constants
    int startIdx = 0;
    int endIdx = 10000;
    float playbackSpeed = 0;    // 0 for linearize (play as fast as possible, while sequentializing tracking & mapping). otherwise, factor on timestamps.

    int lstart = startIdx;
    int lend = endIdx;
    int linc = 1;

    std::shared_ptr<ImageFolderReader> reader = (
        std::make_shared<ImageFolderReader>(ImageFolderReader::TUM_MONO, source, calib, gammaCalib, vignette));
    reader->setGlobalCalibration();

    std::shared_ptr<ORBVocabulary> voc = std::make_shared<ORBVocabulary>();
    voc->load(voc_path);

    std::shared_ptr<FullSystem> fullSystem = std::make_shared<FullSystem>(voc);
    fullSystem->setGammaFunction(reader->getPhotometricGamma());
    fullSystem->linearizeOperation = (playbackSpeed == 0);

    std::shared_ptr<PangolinDSOViewer> viewer =
        std::make_shared<PangolinDSOViewer>(400, 400, false);
    fullSystem->setViewer(viewer);

    bool done = false;
    std::thread runthread([&]() {
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
        double sInitializerOffset = 0;

        for (int ii = 0; ii < (int) idsToPlay.size(); ii++) {
            if (!fullSystem->initialized)    // if not initialized: reset start time.
            {
                gettimeofday(&tv_start, NULL);
                started = clock();
                sInitializerOffset = timesToPlayAt[ii];
            }

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
                fullSystem->addActiveFrame(img, i);
            }
            delete img;

            if (fullSystem->initFailed || setting_fullResetRequested) {
                if (ii < 250 || setting_fullResetRequested) {
                    LOG(INFO) << "Init failed, RESETTING!";
                    fullSystem = std::make_shared<FullSystem>(voc);
                    fullSystem->setGammaFunction(reader->getPhotometricGamma());
                    fullSystem->linearizeOperation = (playbackSpeed == 0);
                    if (viewer) {
                        viewer->reset();
                        sleep(1);
                        fullSystem->setViewer(viewer);
                    }
                    setting_fullResetRequested = false;
                }
            }

            if (fullSystem->isLost) {
                LOG(INFO) << "Lost!";
                break;
            }

            if (done) {
                break;
            }
        }

        fullSystem->blockUntilMappingIsFinished();
        //cout << "Sleeping..." << endl;
        //sleep(10);
        //cout << "Done sleeping" << endl;

        clock_t ended = clock();
        struct timeval tv_end;
        gettimeofday(&tv_end, NULL);

        fullSystem->printResult(output_file, true);

        int numFramesProcessed = abs(idsToPlay[0] - idsToPlay.back());
        double numSecondsProcessed = fabs(reader->getTimestamp(idsToPlay[0]) - reader->getTimestamp(idsToPlay.back()));
        double MilliSecondsTakenSingle = 1000.0f * (ended - started) / (float) (CLOCKS_PER_SEC);
        double MilliSecondsTakenMT = sInitializerOffset + ((tv_end.tv_sec - tv_start.tv_sec) * 1000.0f +
                                                           (tv_end.tv_usec - tv_start.tv_usec) / 1000.0f);
        printf("\n======================"
               "\n%d Frames (%.1f fps)"
               "\n%.2fms per frame (single core); "
               "\n%.2fms per frame (multi core); "
               "\n%.3fx (single core); "
               "\n%.3fx (multi core); "
               "\n======================\n\n",
               numFramesProcessed, numFramesProcessed / numSecondsProcessed,
               MilliSecondsTakenSingle / numFramesProcessed,
               MilliSecondsTakenMT / (float) numFramesProcessed,
               1000 / (MilliSecondsTakenSingle / numSecondsProcessed),
               1000 / (MilliSecondsTakenMT / numSecondsProcessed));
        // if (setting_logStuff) {
        //     std::ofstream tmlog;
        //     tmlog.open("logs/time.txt", std::ios::trunc | std::ios::out);
        //     tmlog << 1000.0f * (ended - started) / (float) (CLOCKS_PER_SEC * reader->getNumImages()) << " "
        //           << ((tv_end.tv_sec - tv_start.tv_sec) * 1000.0f + (tv_end.tv_usec - tv_start.tv_usec) / 1000.0f) /
        //              (float) reader->getNumImages() << "\n";
        //     tmlog.flush();
        //     tmlog.close();
        // }
        cout << "Thread complete!" << endl;
                          });

    std::thread viewer_thread([&]() {
        viewer->run();  // mac os should keep this in main thread.
                          });

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    done = true;
    cout << "Waiting for thread to join..." << endl;
    runthread.join();
    return 0;
}
