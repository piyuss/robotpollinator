#include "Stewart.h"
// Stewart::Stewart(){

// }

// Stewart::~Stewart(){
    
// }
        void Stewart::create_beta_variables(){
            for (int i =0; i< 6; ++i){
                sin_beta[i] = sin(deg_to_rad(beta[i]));
                cos_beta[i] = cos(deg_to_rad(beta[i]));
            }
        }

void Stewart::calculateJointVectors(){
  base_joints[0].x = BASE_RADIUS * cos(deg_to_rad(-60)) + 
                       DISTANCE_ALONG_BASE_EDGE * cos(deg_to_rad(30)) + 
                       OFFSET_FROM_BASE_EDGE * cos(deg_to_rad(-60));
	base_joints[1].x =  BASE_RADIUS * cos(deg_to_rad(60)) + 
                        DISTANCE_ALONG_BASE_EDGE * cos(deg_to_rad(-30)) + 
                        OFFSET_FROM_BASE_EDGE * cos(deg_to_rad(60));
	base_joints[2].x =  BASE_RADIUS * cos(deg_to_rad(60)) + 
                        DISTANCE_ALONG_BASE_EDGE * cos(deg_to_rad(150)) + 
                        OFFSET_FROM_BASE_EDGE * cos(deg_to_rad(60));
	base_joints[3].x =  BASE_RADIUS * cos(deg_to_rad(180)) + 
                        DISTANCE_ALONG_BASE_EDGE * cos(deg_to_rad(90)) + 
                        OFFSET_FROM_BASE_EDGE * cos(deg_to_rad(180));
	base_joints[4].x =  BASE_RADIUS * cos(deg_to_rad(180)) + 
                        DISTANCE_ALONG_BASE_EDGE * cos(deg_to_rad(-90)) + 
                        OFFSET_FROM_BASE_EDGE * cos(deg_to_rad(180));
	base_joints[5].x =  BASE_RADIUS * cos(deg_to_rad(-60)) + 
                        DISTANCE_ALONG_BASE_EDGE * cos(deg_to_rad(-150)) + 
                        OFFSET_FROM_BASE_EDGE * cos(deg_to_rad(-60));
    base_joints[0].y = BASE_RADIUS * sin(deg_to_rad(-60)) + 
                       DISTANCE_ALONG_BASE_EDGE * sin(deg_to_rad(30)) +
                       OFFSET_FROM_BASE_EDGE * sin(deg_to_rad(-60));
    base_joints[1].y = BASE_RADIUS * sin(deg_to_rad(60)) + 
                       DISTANCE_ALONG_BASE_EDGE * sin(deg_to_rad(-30)) + 
                       OFFSET_FROM_BASE_EDGE * sin(deg_to_rad(60));
    base_joints[2].y = BASE_RADIUS * sin(deg_to_rad(60)) + 
                       DISTANCE_ALONG_BASE_EDGE * sin(deg_to_rad(150)) + 
                       OFFSET_FROM_BASE_EDGE * sin(deg_to_rad(60));
    base_joints[3].y = BASE_RADIUS * sin(deg_to_rad(0)) + 
                       DISTANCE_ALONG_BASE_EDGE * sin(deg_to_rad(90)) + 
                       OFFSET_FROM_BASE_EDGE * sin(deg_to_rad(0));
    base_joints[4].y = BASE_RADIUS * sin(deg_to_rad(0)) + 
                       DISTANCE_ALONG_BASE_EDGE * sin(deg_to_rad(-90)) + 
                       OFFSET_FROM_BASE_EDGE * sin(deg_to_rad(0));
    base_joints[5].y = BASE_RADIUS * sin(deg_to_rad(-60)) + 
                       DISTANCE_ALONG_BASE_EDGE * sin(deg_to_rad(-150)) + 
                       OFFSET_FROM_BASE_EDGE * sin(deg_to_rad(-60));
    
    platform_joints[0].x = PLATFORM_RADIUS * cos(deg_to_rad(-60)) + 
                           DISTANCE_ALONG_PLATFORM_EDGE * cos(deg_to_rad(30)) + 
                           OFFSET_FROM_PLATFORM_EDGE * cos(deg_to_rad(-60));
    platform_joints[1].x = PLATFORM_RADIUS * cos(deg_to_rad(60)) + 
                           DISTANCE_ALONG_PLATFORM_EDGE * cos(deg_to_rad(-30)) + 
                           OFFSET_FROM_PLATFORM_EDGE * cos(deg_to_rad(60));
    platform_joints[2].x = PLATFORM_RADIUS * cos(deg_to_rad(60)) + 
                           DISTANCE_ALONG_PLATFORM_EDGE * cos(deg_to_rad(150)) + 
                           OFFSET_FROM_PLATFORM_EDGE * cos(deg_to_rad(60));
    platform_joints[3].x = PLATFORM_RADIUS * cos(deg_to_rad(180)) + 
                           DISTANCE_ALONG_PLATFORM_EDGE * cos(deg_to_rad(90)) + 
                           OFFSET_FROM_PLATFORM_EDGE * cos(deg_to_rad(180));
    platform_joints[4].x = PLATFORM_RADIUS * cos(deg_to_rad(180)) + 
                           DISTANCE_ALONG_PLATFORM_EDGE * cos(deg_to_rad(-90)) + 
                           OFFSET_FROM_PLATFORM_EDGE * cos(deg_to_rad(180));
    platform_joints[5].x = PLATFORM_RADIUS * cos(deg_to_rad(-60)) + 
                           DISTANCE_ALONG_PLATFORM_EDGE * cos(deg_to_rad(-150)) + 
                           OFFSET_FROM_PLATFORM_EDGE * cos(deg_to_rad(-60));

    platform_joints[0].y = PLATFORM_RADIUS * sin(deg_to_rad(-60)) + 
                           DISTANCE_ALONG_PLATFORM_EDGE * sin(deg_to_rad(30)) + 
                           OFFSET_FROM_PLATFORM_EDGE * sin(deg_to_rad(-60));
    platform_joints[1].y = PLATFORM_RADIUS * sin(deg_to_rad(60)) + 
                           DISTANCE_ALONG_PLATFORM_EDGE * sin(deg_to_rad(-30)) + 
                           OFFSET_FROM_PLATFORM_EDGE * sin(deg_to_rad(60));
    platform_joints[2].y = PLATFORM_RADIUS * sin(deg_to_rad(60)) + 
                           DISTANCE_ALONG_PLATFORM_EDGE * sin(deg_to_rad(150)) + 
                           OFFSET_FROM_PLATFORM_EDGE * sin(deg_to_rad(60));
    platform_joints[3].y = PLATFORM_RADIUS * sin(deg_to_rad(0)) + 
                           DISTANCE_ALONG_PLATFORM_EDGE * sin(deg_to_rad(90)) + 
                           OFFSET_FROM_PLATFORM_EDGE * sin(deg_to_rad(0));
    platform_joints[4].y = PLATFORM_RADIUS * sin(deg_to_rad(0)) + 
                           DISTANCE_ALONG_PLATFORM_EDGE * sin(deg_to_rad(-90)) + 
                           OFFSET_FROM_PLATFORM_EDGE * sin(deg_to_rad(0));
    platform_joints[5].y = PLATFORM_RADIUS * sin(deg_to_rad(-60)) + 
                           DISTANCE_ALONG_PLATFORM_EDGE * sin(deg_to_rad(-150)) + 
                           OFFSET_FROM_PLATFORM_EDGE * sin(deg_to_rad(-60));

}

void Stewart::getPose(){
    target_pose.x = x_translation;
    target_pose.y = y_translation;
    target_pose.z = z_translation;

    target_pose.theta = roll;
    target_pose.phi = pitch;
    target_pose.psi = yaw;

}


void Stewart::calculateServoAngles(){

  quaternion rotation_quaternion = quaternionFromEuler(rotation);

  vector rotated_vector[6];

  for (int leg_num = 0; leg_num < 6; ++leg_num){
    rotated_vector[leg_num] = Stewart::rotateVector(rotation_quaternion, platform_joints[leg_num]);
    platform_joint_vector[leg_num].x = rotated_vector[leg_num].x + translation.x;
    platform_joint_vector[leg_num].y = rotated_vector[leg_num].y + translation.y;
    platform_joint_vector[leg_num].z = rotated_vector[leg_num].z + translation.z;

    //print_vector(platform_joint_vector[leg_num]);
  }
  

  for (int servo_index = 0; servo_index < 6; ++servo_index){
      Stewart::vector Li;

      double min = deg_to_rad(SERVO_MIN);
      double max = deg_to_rad(SERVO_MAX);
      
      Li.x = platform_joint_vector[servo_index].x - base_joints[servo_index].x;
      Li.y = platform_joint_vector[servo_index].y - base_joints[servo_index].y;
      Li.z = platform_joint_vector[servo_index].z - base_joints[servo_index].z;
      
      //print_vector(Li);

      double gk = Li.x * Li.x + Li.y * Li.y + Li.z * Li.z - PLATFORM_LINK_LENGTH * PLATFORM_LINK_LENGTH + SERVO_ARM_LENGTH * SERVO_ARM_LENGTH;
      double ek = 2 * SERVO_ARM_LENGTH * Li.z;
      double fk = 2 * SERVO_ARM_LENGTH * (cos_beta[servo_index] * Li.x + sin_beta[servo_index] * Li.y);
      
      vector H[6];
      vector alpha[6];

      double sq_sum = ek * ek + fk * fk;
      double sqrt1 = sqrt(1 - gk * gk / sq_sum);
      double sqrt2 = sqrt(sq_sum);
      double sin_alpha = (gk * ek) / sq_sum - (fk * sqrt1) / sqrt2;
      double cos_alpha = (gk * fk) / sq_sum + (ek * sqrt1) / sqrt2;
      
      
      H[servo_index].x = base_joints[servo_index].x + SERVO_ARM_LENGTH * cos_alpha * cos_beta[servo_index];
      H[servo_index].y = base_joints[servo_index].y + SERVO_ARM_LENGTH * cos_alpha * sin_beta[servo_index];
      H[servo_index].z = base_joints[servo_index].z + SERVO_ARM_LENGTH * sin_alpha;
       
      //print_vector(H[servo_index]);

      double angle_radians= asin((H[servo_index].z - base_joints[servo_index].z) / SERVO_ARM_LENGTH);

      angles[servo_index] = rad_to_deg(angle_radians);


          if (isnan(angles[servo_index])){
            //Rod too short
            angles[servo_index] = 0;
          }
          
          else if (not(SERVO_MIN <= angles[servo_index] and angles[servo_index] <= SERVO_MAX)){

            angles[servo_index] = 0;
          
          }
       std::cout<< angles[servo_index] << std::endl;

  }

}

