#include "analytics.h"


struct analytics_t analytics_init(void)
{
    struct analytics_t out;

    out.powerpose_count = 0;
    out.t_count = 0;
    out.victory_count = 0;
    out.usain_count = 0;
    out.stop_count = 0;
    out.flying_count = 0;
    out.waving_r_count = 0;
    out.waving_l_count = 0;
    out.pointing_trf_count = 0;
    out.pointing_rf_count = 0;
    out.pointing_tlf_count = 0;
    out.pointing_lf_count = 0;
    out.pointing_tr_count = 0;
    out.pointing_r_count = 0;
    out.pointing_tl_count = 0;
    out.pointing_l_count = 0;

    return out;
}

void updateAnalytics(struct analytics_t counts) {
    int result; // file renaming and removing error checking
    char filename[] = "/home/capstone38/Desktop/electricTree/analytics.txt";
    char temp_filename[] = "/home/capstone38/Desktop/electricTree/temp_analytics.txt";

    // Update Analytics
    if(ENABLE_ANALYTICS == true) {
        bool error_detected_in = false;
        bool error_detected_out = false;

        ifstream inFile;
        inFile.open(filename);
        if(!inFile.is_open()) {
            error_detected_in = true;
        }

        ofstream outFile;
        outFile.open(temp_filename);
        if(!outFile.is_open()) {
            error_detected_out = true;
        }

        if(error_detected_in == false && error_detected_out == true) inFile.close();
        if(error_detected_in == true && error_detected_out == false) outFile.close();

        if(error_detected_in == false && error_detected_out == false) {

            inFile >> format;
            outFile << format << endl;

            inFile >> format;
            outFile << format << endl;


            while(inFile >> gesture >> gestureCount >> totalCount) {
                if(gesture == "USAIN:"){
                    totalCount += counts.usain_count;
                    gestureCount = counts.usain_count;
                }
                else if (gesture == "FLEXING:") {
                    totalCount += counts.powerpose_count;
                    gestureCount = counts.powerpose_count;
                }
                else if (gesture == "VICTORY:") {
                    totalCount += counts.victory_count;
                    gestureCount = counts.victory_count;
                }
                else if (gesture == "TPOSE:") {
                    totalCount += counts.t_count;
                    gestureCount = counts.t_count;
                }
                else if (gesture == "STOP:") {
                    totalCount += counts.stop_count;
                    gestureCount = counts.stop_count;
                }
                else if (gesture == "POINTING_TRF:") {
                    totalCount += counts.pointing_trf_count;
                    gestureCount = counts.pointing_trf_count;
                }
                else if (gesture == "POINTING_RF:") {
                    totalCount += counts.pointing_rf_count;
                    gestureCount = counts.pointing_rf_count;
                }
                else if (gesture == "POINTING_TLF:") {
                    totalCount += counts.pointing_tlf_count;
                    gestureCount = counts.pointing_tlf_count;
                }
                else if (gesture == "POINTING_LF:") {
                    totalCount += counts.pointing_lf_count;
                    gestureCount = counts.pointing_lf_count;
                }
                else if (gesture == "POINTING_TR:") {
                    totalCount += counts.pointing_tr_count;
                    gestureCount = counts.pointing_tr_count;
                }
                else if (gesture == "POINTING_R:") {
                    totalCount += counts.pointing_r_count;
                    gestureCount = counts.pointing_r_count;
                }
                else if (gesture == "POINTING_TL:") {
                    totalCount += counts.pointing_tl_count;
                    gestureCount = counts.pointing_tl_count;
                }
                else if (gesture == "POINTING_L:") {
                    totalCount += counts.pointing_l_count;
                    gestureCount = counts.pointing_l_count;
                }
                else if (gesture == "FLYING:") {
                    totalCount += counts.flying_count;
                    gestureCount = counts.flying_count;
                }
                else if (gesture == "WAVING_R:") {
                    totalCount += counts.waving_r_count;
                    gestureCount = counts.waving_r_count;
                }
                else if (gesture == "WAVING_L:") {
                    totalCount += counts.waving_l_count;
                    gestureCount = counts.waving_l_count;
                }

                // Present day gestures count
                outFile << gesture << " " << gestureCount << " ";


                // Total Count of Gestures to date
                outFile << total << " " << totalCount << endl;
            }

            inFile.close();
            outFile.close();

            result = remove(filename);
            if(result != 0)
                perror("Error deleting file");
            else
                puts("File successfully deleted");

            result = rename(temp_filename, filename);
            if(result != 0)
                perror("Error renaming file");
            else
                puts("File successfully renamed");
        }
    }
}
