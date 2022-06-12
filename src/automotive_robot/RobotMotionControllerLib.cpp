#include "automotive_robot_headers/RobotController.hpp"

/******************************************************************************************************/
/* functions for smooth asp movement */
/*
inline void printOperationInformation(double direction, unsigned elapsedTime){
    return;
    cout << state;
    // cout << "   toc do di chuyen: " << actual_odom_velocity << "   message: " << vel_msg.linear.x;
    // cout << "     toc do quay: " << z/M_PI*180 << "   message: " << vel_msg.angular.z/M_PI*180;
    cout << "     huong diem tiep theo: " << direction/M_PI*180 << "   huong robot: " << odomAngleXaxis/M_PI*180 << 
    "   do lech: " << -(direction - odomAngleXaxis)/M_PI*180 << "   toc do can chinh: " << vel_msg.angular.z/M_PI*180 <<  "\n";
    //cout << "   elapsed: " << elapsedTime << "\n";
}

void calculate_start_bending_vector_and_stop_bending_vector(const automotive_robot::Path::ConstPtr &msg, 
    float *vector_nextPoint_to_start_bending_point, float *vector_nextPoint_to_stop_bending_point, const float start_bending_distance,int i){
    vector_nextPoint_to_start_bending_point[0] = msg->points[i - 1].x - msg->points[i].x; 
    vector_nextPoint_to_start_bending_point[1] = msg->points[i - 1].y - msg->points[i].y;
    vector_nextPoint_to_stop_bending_point[0] = msg->points[i + 1].x - msg->points[i].x; 
    vector_nextPoint_to_stop_bending_point[1] = msg->points[i + 1].y - msg->points[i].y;
    
    float distance_from_nextPoint_to_start_point = sqrt(vector_nextPoint_to_start_bending_point[0]*vector_nextPoint_to_start_bending_point[0] 
        + vector_nextPoint_to_start_bending_point[1]*vector_nextPoint_to_start_bending_point[1]);
    float distance_from_nextPoint_to_end_point = sqrt(vector_nextPoint_to_stop_bending_point[0]*vector_nextPoint_to_stop_bending_point[0] 
        + vector_nextPoint_to_stop_bending_point[1]*vector_nextPoint_to_stop_bending_point[1]);
    
    vector_nextPoint_to_start_bending_point[0] = vector_nextPoint_to_start_bending_point[0]/distance_from_nextPoint_to_start_point*start_bending_distance;
    vector_nextPoint_to_start_bending_point[1] = vector_nextPoint_to_start_bending_point[1]/distance_from_nextPoint_to_start_point*start_bending_distance;

    vector_nextPoint_to_stop_bending_point[0] = vector_nextPoint_to_stop_bending_point[0]/distance_from_nextPoint_to_end_point*start_bending_distance;
    vector_nextPoint_to_stop_bending_point[1] = vector_nextPoint_to_stop_bending_point[1]/distance_from_nextPoint_to_end_point*start_bending_distance;
}

void translation_of_point(const float *translationary_vector, const automotive_robot::Point &toBe_translated_point, automotive_robot::Point &translated_point){
    translated_point.x = toBe_translated_point.x + translationary_vector[0];
    translated_point.y = toBe_translated_point.y + translationary_vector[1];
}
*/
/******************************************************************************************************/
