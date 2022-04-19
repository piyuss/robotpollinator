
#pragma once
// #ifndef _STEWART_H
// #define _STEWART_H
#include <math.h>
#include <iostream>
//#include <Arduino.h>
#include "Geometry.h"

class Stewart{
    public:
        // Stewart();

        // virtual ~Stewart();
        
        typedef struct pose {
        float x;
        float y;
        float z;

        float theta;
        float phi;
        float psi;
        } pose;

        typedef struct vector {
            float x = 0;
            float y = 0;
            float z = 0;
        } vector;

        typedef struct euler {
            float theta;
            float phi;
            float psi;
        } euler;

        typedef struct quaternion{
            float w;
            float x;
            float y;
            float z;

        } quaternion;


        double angles[6];
        double beta[6] = {30, -30, 150, 90, -90, -150};
        double sin_beta[6];
        double cos_beta[6];
        void create_beta_variables();
        


        pose target_pose;
        vector translation;
        euler rotation;
        
        //const double axisOrientation[6];
        float servo_angles[6];

        vector base_joints[6];
        vector platform_joints[6];
        vector platform_joint_vector[6];
        
        void calculateJointVectors();
        void getPose();
        void calculateServoAngles();

        float x_translation;
        float y_translation;
        float z_translation;
        float roll;
        float pitch;
        float yaw;


       void print_vector(vector v){
           std::cout << v.x <<","<< v.y << "," << v.z << std::endl;
       }


        quaternion quaternionConjugate(quaternion q){

            quaternion conjugate_quat = {0.0, 0.0, 0.0, 0.0};

            conjugate_quat.x = -q.x;
            conjugate_quat.y = -q.y;
            conjugate_quat.z = -q.z;
            conjugate_quat.w = q.w;
            
            return conjugate_quat;
        };

        quaternion multiplyQuaternions(quaternion q1, quaternion q2){

            quaternion product_quat = {0.0, 0.0, 0.0, 0.0};

            product_quat.x =  q1.x * q2.w + q1.y * q2.z - q1.z * q2.y + q1.w * q2.x;
            product_quat.y = -q1.x * q2.z + q1.y * q2.w + q1.z * q2.x + q1.w * q2.y;
            product_quat.z =  q1.x * q2.y - q1.y * q2.x + q1.z * q2.w + q1.w * q2.z;
            product_quat.w = -q1.x * q2.x - q1.y * q2.y - q1.z * q2.z + q1.w * q2.w;
            
            return product_quat;
        };

        vector rotateVector(quaternion q, vector v){

            quaternion temp_quat = {0, v.x, v.y, v.z};
            quaternion q_conjugate = quaternionConjugate(q);
            quaternion quat_product = multiplyQuaternions(multiplyQuaternions(q, temp_quat), q_conjugate);  
            vector rotated_vec = {quat_product.x, quat_product.y, quat_product.z};
            return rotated_vec; 
        };

        quaternion quaternionFromEuler(euler e){
            float X = deg_to_rad(e.theta);
            float Y = deg_to_rad(e.phi);
            float Z = deg_to_rad(e.psi);

            X *= 0.5f;
            Y *= 0.5f;
            Z *= 0.5f;

            float sinx = sinf(X);
            float siny = sinf(Y);
            float sinz = sinf(Z);
            float cosx = cosf(X);
            float cosy = cosf(Y);
            float cosz = cosf(Z);

            quaternion q;

            q.w = cosx * cosy * cosz + sinx * siny * sinz;
            q.x = sinx * cosy * cosz - cosx * siny * sinz;
            q.y = cosx * siny * cosz + sinx * cosy * sinz;
            q.z = cosx * cosy * sinz - sinx * siny * cosz;

            return q;
        };

        inline double rad_to_deg(const double & radian_value){
            return radian_value * 180.0 / M_PI;
        };

        inline double deg_to_rad(const double & degree_value){
            return degree_value * M_PI / 180.0;
        };
};

// #endif


