/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __TASKS_H__
#define __TASKS_H__

#include <unistd.h>
#include <iostream>

#include <sys/mman.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <alchemy/mutex.h>
#include <alchemy/sem.h>
#include <alchemy/queue.h>

#include "messages.h"
#include "commonitor.h"
#include "comrobot.h"
#include "camera.h"
#include "img.h"

using namespace std;

class Tasks {
public:
    /**
     * @brief Initializes main structures (semaphores, tasks, mutex, etc.)
     */
    void Init();

    /**
     * @brief Starts tasks
     */
    void Run();

    /**
     * @brief Stops tasks
     */
    void Stop();
    
    /**
     * @brief Suspends main thread
     */
    void Join();
    
private:
    /**********************************************************************/
    /* Shared data                                                        */
    /**********************************************************************/
    ComMonitor monitor;
    ComRobot robot;
    int robotStarted = 0;
    bool status_cam = false;
    int move = MESSAGE_ROBOT_STOP;
    int validation=0;

    Arena arena_save;

    Camera cam;

    Img * img_cam;
    
    int stop = 0;
    
    int validated_and_save = 0;
    
    int findPosition=0;
    
    int pos_valid=0;
    
    /**********************************************************************/
    /* Tasks                                                              */
    /**********************************************************************/
    RT_TASK th_server;
    RT_TASK th_sendToMon;
    RT_TASK th_receiveFromMon;
    RT_TASK th_openComRobot;
    RT_TASK th_startRobot;
    RT_TASK th_move;
    RT_TASK th_checkBattery;
    RT_TASK th_openCamera;
    RT_TASK th_closeCamera;
    RT_TASK th_takePictures;
    RT_TASK th_computePosition;
    RT_TASK th_calibrateArena;

    /**********************************************************************/
    /* Mutex                                                              */
    /**********************************************************************/
    RT_MUTEX mutex_monitor;
    RT_MUTEX mutex_robot;
    RT_MUTEX mutex_robotStarted;
    RT_MUTEX mutex_move;
    RT_MUTEX mutex_camera;
    RT_MUTEX mutex_status_cam;
    RT_MUTEX mutex_image;
    RT_MUTEX mutex_arena;
    RT_MUTEX mutex_stop;
    RT_MUTEX mutex_validation;
    RT_MUTEX mutex_valid_save;
    RT_MUTEX mutex_findPos;
    RT_MUTEX mutex_validPos;

    /**********************************************************************/
    /* Semaphores                                                         */
    /**********************************************************************/
    RT_SEM sem_barrier;
    RT_SEM sem_openComRobot;
    RT_SEM sem_serverOk;
    RT_SEM sem_startRobot;
    RT_SEM sem_getBattery;

    RT_SEM sem_openCam;
    RT_SEM sem_closeCam;

    RT_SEM sem_findPosition;
    RT_SEM sem_stopPosition;

    //Semaphores liés a calibration de l'arene
    RT_SEM sem_findArena;
    RT_SEM sem_stopPeriodicImg;
    RT_SEM sem_restartPeriodicImg;
    RT_SEM sem_Arena;
    //RT_SEM sem_unvalidatedArena;
    

    /**********************************************************************/
    /* Message queues                                                     */
    /**********************************************************************/
    int MSG_QUEUE_SIZE;
    RT_QUEUE q_messageToMon;
    
    /**********************************************************************/
    /* Tasks' functions                                                   */
    /**********************************************************************/
    /**
     * @brief Thread handling server communication with the monitor.
     */
    void ServerTask(void *arg);
     
    /**
     * @brief Thread sending data to monitor.
     */
    void SendToMonTask(void *arg);
        
    /**
     * @brief Thread receiving data from monitor.
     */
    void ReceiveFromMonTask(void *arg);
    
    /**
     * @brief Thread opening communication with the robot.
     */
    void OpenComRobot(void *arg);

    /**
     * @brief Thread starting the communication with the robot.
     */
    void StartRobotTask(void *arg);
    
    /**
     * @brief Thread handling control of the robot.
     */
    void MoveTask(void *arg);
    
    /**
     * @brief Thread handling the level of battery.
     */
    void CheckBattery(void *arg);
    
     /**
     * @brief Thread handling the managament of the camera
     */
    void openCameraTask(void *arg);
    
         /**
     * @brief Thread handling the managament of the camera
     */
    void closeCameraTask(void *arg);
    
    /**
     * @brief Thread handling the taking and sending of the picture , periodicly
     */
    void takePicturesTask(void *arg);

     /**
    * @brief Thread handling the finding, drawing and send the robot's position
    */
    void computePositionTask(void *arg);

    /**
    * @brief Thread the calibration and validation of the arena
    */
    void calibrateArenaTask(void *arg);


    
    /**********************************************************************/
    /* Queue services                                                     */
    /**********************************************************************/
    /**
     * Write a message in a given queue
     * @param queue Queue identifier
     * @param msg Message to be stored
     */
    void WriteInQueue(RT_QUEUE *queue, Message *msg);
    
    /**
     * Read a message from a given queue, block if empty
     * @param queue Queue identifier
     * @return Message read
     */
    Message *ReadInQueue(RT_QUEUE *queue);

};

#endif // __TASKS_H__ 