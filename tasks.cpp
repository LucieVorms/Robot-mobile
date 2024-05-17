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

#include "tasks.h"
#include <stdexcept>

// Déclaration des priorités des taches
#define PRIORITY_TSERVER 30
#define PRIORITY_TOPENCOMROBOT 20
#define PRIORITY_TMOVE 22
#define PRIORITY_TSENDTOMON 23
#define PRIORITY_TRECEIVEFROMMON 25
#define PRIORITY_TSTARTROBOT 20
#define PRIORITY_TCAMERA 21
#define PRIORITY_TPICTURES 21
#define PRIORITY_TBATTERY 22
#define PRIORITY_TPOSITION 21
#define PRIORITY_TARENA 21


/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 * 
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 * 
 * 3- Data flow is probably not optimal
 * 
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 * 
 * 5- Same behavior existe for ComMonitor::Write !
 * 
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 * 
 * 7- Good luck !
 */

/**
 * @brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void Tasks::Init() {
    int status;
    int err;

    /**************************************************************************************/
    /* 	Mutex creation                                                                    */
    /**************************************************************************************/
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_camera, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_image, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_arena, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_status_cam, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_stop, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_validation, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_valid_save, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_findPos, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_validPos, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Mutexes created successfully" << endl << flush;

    /**************************************************************************************/
    /* 	Semaphors creation       							  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_getBattery, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openCam, NULL, 0, S_FIFO)){
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_closeCam, NULL, 0, S_FIFO)){
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_findPosition, NULL, 0, S_FIFO)){
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_stopPosition, NULL, 0, S_FIFO)){
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_findArena, NULL, 0, S_FIFO)){
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_stopPeriodicImg, NULL, 0, S_FIFO)){
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_restartPeriodicImg, NULL, 0, S_FIFO)){
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_Arena, NULL, 0, S_FIFO)){
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }



    cout << "Semaphores created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************************************************************************/
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_checkBattery, "th_checkBattery", 0, PRIORITY_TBATTERY, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openCamera, "th_openCamera", 0, PRIORITY_TCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_closeCamera, "th_closeCamera", 0, PRIORITY_TCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_takePictures, "th_takePictures", 0, PRIORITY_TPICTURES, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_computePosition, "th_computePosition", 0, PRIORITY_TPOSITION, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_calibrateArena, "th_calibrateArena", 0, PRIORITY_TARENA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Queues created successfully" << endl << flush;

}

/**
 * @brief Démarrage des tâches
 */
void Tasks::Run() {
    rt_task_set_priority(NULL, T_LOPRIO);
    int err;

    if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, (void(*)(void*)) & Tasks::SendToMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openComRobot, (void(*)(void*)) & Tasks::OpenComRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_checkBattery, (void(*)(void*)) & Tasks::CheckBattery, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openCamera, (void(*)(void*)) & Tasks::openCameraTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_closeCamera, (void(*)(void*)) & Tasks::closeCameraTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_takePictures, (void(*)(void*)) & Tasks::takePicturesTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_computePosition, (void(*)(void*)) & Tasks::computePositionTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_calibrateArena, (void(*)(void*)) & Tasks::calibrateArenaTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
    monitor.Close();
    robot.Close();
}

/**
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier);
    pause();
}

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task server starts here                                                        */
    /**************************************************************************************/
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    status = monitor.Open(SERVER_PORT);
    rt_mutex_release(&mutex_monitor);

    cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

    if (status < 0) throw std::runtime_error {
        "Unable to start server on port " + std::to_string(SERVER_PORT)
    };
    monitor.AcceptClient(); // Wait the monitor client
    cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
    rt_sem_broadcast(&sem_serverOk);
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
    Message *msg;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);

    while (1) {
        cout << "wait msg to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);
        cout << "Send msg to mon: " << msg->ToString() << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msg); // The message is deleted with the Write
        rt_mutex_release(&mutex_monitor);
    }
}

/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task receiveFromMon starts here                                                */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "Received message from monitor activated" << endl << flush;

    while (1) {
        msgRcv = monitor.Read();
        cout << "Rcv <= " << msgRcv->ToString() << endl << flush;

        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
            delete(msgRcv);
            exit(-1);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
            rt_sem_v(&sem_openComRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
            rt_sem_v(&sem_startRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = msgRcv->GetID();
            rt_mutex_release(&mutex_move);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_BATTERY_GET)){
            rt_sem_v(&sem_getBattery);
        }

        else if(msgRcv->CompareID(MESSAGE_CAM_CLOSE)){
            rt_sem_v(&sem_closeCam);
        }
        else if(msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_START)){
            
            rt_mutex_acquire(&mutex_findPos, TM_INFINITE);
            findPosition = 1;
            cout<< "findPos = "<<findPosition;
            rt_mutex_release(&mutex_findPos);
        }
        else if(msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_STOP)){
            rt_mutex_acquire(&mutex_findPos, TM_INFINITE);
            findPosition = 0;
            cout<< "findPos = "<<findPosition;
            rt_mutex_release(&mutex_findPos);
        }
        else if(msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA)){
            rt_sem_v(&sem_findArena);
        }
        else if(msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM)) 
                
        {
            rt_mutex_acquire(&mutex_validation, TM_INFINITE);
            validation = 1;
            cout<< "validation = "<<validation;
            rt_mutex_release(&mutex_validation);
            rt_sem_v(&sem_Arena);

        }
        else if(msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM)){
            rt_mutex_acquire(&mutex_validation, TM_INFINITE);
            validation = 0;
            cout<< "validation = "<<validation;
            rt_mutex_release(&mutex_validation);
            rt_sem_v(&sem_Arena);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_OPEN)){
            rt_sem_v(&sem_openCam);
        }
        delete(msgRcv); // mus be deleted manually, no consumer
    }
}

/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
    int status;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;

        Message * msgSend;
        if (status < 0) {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
    }
}

/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {

        Message * msgSend;
        rt_sem_p(&sem_startRobot, TM_INFINITE);
        cout << "Start robot without watchdog (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        msgSend = robot.Write(robot.StartWithoutWD());
        rt_mutex_release(&mutex_robot);
        cout << msgSend->GetID();
        cout << ")" << endl;

        cout << "Movement answer: " << msgSend->ToString() << endl << flush;
        WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon

        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
        }
    }
}

/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    int rs;
    int cpMove;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {
        rt_task_wait_period(NULL);
        cout << "Periodic movement update";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);
            
            cout << " move: " << cpMove;
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            robot.Write(new Message((MessageID)cpMove));
            rt_mutex_release(&mutex_robot);
        }
        cout << endl << flush;
    }
}


/**
 * @brief Thread handling the level of robot.
 */
void Tasks::CheckBattery(void *arg) {
    MessageBattery * msg;
    int rs;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    // Synchronization (waiting that the monitor asks for the battery level
     rt_sem_p(&sem_getBattery, TM_INFINITE);
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 500000000);

    while (1) {
        rt_task_wait_period(NULL);
        cout << "Periodic checking battery update";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            
            //On récupère le niveau de la batterie
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msg = (MessageBattery*)robot.Write(new Message(MESSAGE_ROBOT_BATTERY_GET));
            rt_mutex_release(&mutex_robot);
            
            //Sending the level to the monitor
            WriteInQueue(&q_messageToMon, msg); 
            
        }
        cout << endl << flush;
    }
}

/**
 * @brief Thread handling control of the robot.
 */
void Tasks::openCameraTask(void *arg) {
    bool cam_open_result;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task manageCamera starts here                                                    */
    /**************************************************************************************/
    while (1) {
        Message * msgSend;
        rt_sem_p(&sem_openCam, TM_INFINITE);
        cout << "Open camera (\n";
        
        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
        cam_open_result = cam.Open();
        rt_mutex_release(&mutex_camera);
        
        rt_mutex_acquire(&mutex_status_cam, TM_INFINITE);
        status_cam=1;
        rt_mutex_release(&mutex_status_cam);
        
        cout << ")" << endl << flush;

        if (cam_open_result==false) {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
        

    }
    
}


void Tasks::closeCameraTask(void *arg) {
    int cam_close_result;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task manageCamera starts here                                                    */
    /**************************************************************************************/
    while (1) {
        Message * msgSend;
        rt_sem_p(&sem_closeCam, TM_INFINITE);
        cout << "Close camera (\n";
        
        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
        cam.Close();
        rt_mutex_release(&mutex_camera);
        
        rt_mutex_acquire(&mutex_status_cam, TM_INFINITE);
        status_cam=0;
        cam_close_result=status_cam;
        rt_mutex_release(&mutex_status_cam);

    
        cout << ")" << endl << flush;

        if (cam_close_result==1) {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
        

    }


}


void Tasks::takePicturesTask(void *arg)
{
    //Variable de lecture de la variable partagee status_cam
    bool sc;

    //Image à envoyer
    Img * img_send;
    
    int stop_periodic_image=0;
    int vs = 0;
    int pos=0;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task takePicture starts here                                                    */
    /**************************************************************************************/

    rt_task_set_periodic(NULL, TM_NOW, 100000000);
   

    while (1) { 
        rt_task_wait_period(NULL);
        
        rt_mutex_acquire(&mutex_stop, TM_INFINITE);
        stop_periodic_image=stop;
        cout<<"Stop periodique debut="<<stop<<endl;
        rt_mutex_release(&mutex_stop);
        
        if (stop_periodic_image==0)
        {
            //Creation d'un message pour envoyer l'image
            MessageImg *msgImg;

          
            rt_mutex_acquire(&mutex_stop, TM_INFINITE);
            stop_periodic_image=stop;
            rt_mutex_release(&mutex_stop);

            cout<<"Stop periodique ="<<stop_periodic_image<<endl;
            cout << "Periodic taking and send picture update";

            rt_mutex_acquire(&mutex_status_cam, TM_INFINITE);
            sc = status_cam;
            rt_mutex_release(&mutex_status_cam);

            //prise de photo et envoie
            if (sc == 1)
            {
                rt_mutex_acquire(&mutex_camera, TM_INFINITE);
                rt_mutex_acquire(&mutex_image, TM_INFINITE);
                cout << "Prise de photo"<<endl;
                img_cam = new Img(cam.Grab());
                rt_mutex_release(&mutex_camera);
                rt_mutex_release(&mutex_image);
                
                rt_mutex_acquire(&mutex_valid_save, TM_INFINITE);
                vs=validated_and_save;
                cout<<"validated and save="<<vs<<endl;
                rt_mutex_release(&mutex_valid_save);
                
                if (vs==1){
                    cout<<"Dans if vs"<<endl;
                    rt_mutex_acquire(&mutex_arena, TM_INFINITE);
                    rt_mutex_acquire(&mutex_image, TM_INFINITE);
                    img_cam->DrawArena(arena_save);
                    cout<<"Dessin peridodique"<<endl;
                    rt_mutex_release(&mutex_image);
                    rt_mutex_release(&mutex_arena);
                }
                
                rt_mutex_acquire(&mutex_image, TM_INFINITE);
                img_send=img_cam;
                rt_mutex_release(&mutex_image);
                
                rt_mutex_acquire(&mutex_validPos, TM_INFINITE);
                pos=pos_valid;
                cout<<"pos valid="<<pos<<endl;
                rt_mutex_release(&mutex_validPos);

                if (pos==0){
                msgImg = new MessageImg(MESSAGE_CAM_IMAGE, img_send);
                WriteInQueue(&q_messageToMon, msgImg); // msgImg will be deleted by sendToMon
                cout << "msg photo classique envoye" << endl;
                }
            }
        }
    }
}



void Tasks::calibrateArenaTask(void *arg)
{
    int arena_validated=0;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task calibrateArenaTask starts here                                           */
    /**************************************************************************************/
    while(1){
        Message * msgSend;
        MessageImg *msgImg;
        Img * img_arene;
        Arena arena;

        rt_sem_p(&sem_findArena, TM_INFINITE);
        //rt_sem_v(&sem_stopPeriodicImg);
        rt_mutex_acquire(&mutex_stop, TM_INFINITE);
        stop=1;
        cout<<"Stop ds arene="<<stop<<endl;
        rt_mutex_release(&mutex_stop);

        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
        rt_mutex_acquire(&mutex_image, TM_INFINITE);
        img_cam = new Img(cam.Grab());
        img_arene = img_cam;
        cout<<"Photo arene prise"<<endl;
        rt_mutex_release(&mutex_camera);
        rt_mutex_release(&mutex_image);

        arena = img_arene->SearchArena();
        if (arena.IsEmpty())
        {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
            cout << "Echec : Arene non trouvée" << endl;
        }
        else
        {
            cout << "Arène trouvée" << endl;
            //Dessin de l'arène sur l'image et envoie
            img_arene->DrawArena(arena);
            cout<<"dessin"<<endl;
            
            msgImg = new MessageImg(MESSAGE_CAM_IMAGE, img_arene);
            WriteInQueue(&q_messageToMon, msgImg);
            cout<<"Image arene+dessin envoyée"<<endl;
            
            //Validation de l'arène
            rt_sem_p(&sem_Arena, TM_INFINITE);
            
            rt_mutex_acquire(&mutex_validation, TM_INFINITE);
            arena_validated = validation;
            rt_mutex_release(&mutex_validation);
            cout <<"Arena validated ="<< arena_validated<< endl;

            if (arena_validated == 1){
                rt_mutex_acquire(&mutex_arena, TM_INFINITE);
                cout << "Arène validée" << endl;
                arena_save = arena; //Sauvegarde de l'arene
                rt_mutex_release(&mutex_arena);
                
                rt_mutex_acquire(&mutex_valid_save, TM_INFINITE);
                validated_and_save=1;
                cout<<"Validated and save ds arena="<<validated_and_save<<endl;
                rt_mutex_release(&mutex_valid_save);

            } else {

                cout << "Arene non validée" << endl;

            }
        }

        rt_mutex_acquire(&mutex_stop, TM_INFINITE);
        stop=0;
        cout<<"Stop a 0="<<stop<<endl;
        rt_mutex_release(&mutex_stop);
        cout<<"Sortie if"<<endl;
    }

}


void Tasks::computePositionTask(void *arg)
{
    int fp=0;
    std::list<Position> LesPositionsRobot; // ou std::list<Position> PositionRobot?
    Position PositionRobot;
    Position PositionNulle;
   

    MessagePosition *msgPos;
    MessageImg *msgImg;
    Img * img_send;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task computePositionTask starts here                                                    */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);
    
    while(1){
    
    rt_task_wait_period(NULL);
    rt_mutex_acquire(&mutex_findPos, TM_INFINITE);
    fp=findPosition;
    cout<<"Find position"<<fp<<endl;
    rt_mutex_release(&mutex_findPos);
   
    if (fp==1)
    {

//        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
//        cout << "Prise de photo" << endl;
//        img = new Img(cam->Grab());
//        rt_mutex_release(&mutex_camera);
        
        rt_mutex_acquire(&mutex_image, TM_INFINITE);
        rt_mutex_acquire(&mutex_arena, TM_INFINITE);
        cout << "Calcul de la position" << endl;
        LesPositionsRobot = img_cam->SearchRobot(arena_save);
        rt_mutex_release(&mutex_arena);
        rt_mutex_release(&mutex_image);

        if (not LesPositionsRobot.empty())
        {
            cout << "Robot trouvé" << endl;
            PositionRobot = LesPositionsRobot.back();
            rt_mutex_acquire(&mutex_image, TM_INFINITE);
            cout << "Dessiner robot" << endl;
            img_cam->DrawRobot(PositionRobot);
            rt_mutex_release(&mutex_image);
            
            msgPos = new MessagePosition(MESSAGE_CAM_POSITION, PositionRobot);
            WriteInQueue(&q_messageToMon, msgPos);
            
            rt_mutex_acquire(&mutex_validPos, TM_INFINITE);
            pos_valid=1;
            rt_mutex_release(&mutex_validPos);
        }
        else
        {
            cout << "Position nulle" << endl;
            //PositionRobot = LesPositionsRobot.front();
            LesPositionsRobot.back().robotId=-1;
            LesPositionsRobot.back().angle=-1;
            LesPositionsRobot.back().center=cv::Point2f(-1.0,-1.0);
            LesPositionsRobot.back().direction=cv::Point2f(-1.0,-1.0);
            
            msgPos = new MessagePosition(MESSAGE_CAM_POSITION, LesPositionsRobot.back());
            WriteInQueue(&q_messageToMon, msgPos);   
        }
        
        rt_mutex_acquire(&mutex_image, TM_INFINITE);
        img_send=img_cam;
        rt_mutex_release(&mutex_image);
        
        msgImg = new MessageImg(MESSAGE_CAM_IMAGE, img_send);
        WriteInQueue(&q_messageToMon, msgImg);
    }
    }
}


/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
    int err;
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
    int err;
    Message *msg;

    if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in read in queue"};
    }/** else {
        cout << "@msg :" << msg << endl << flush;
    } /**/

    return msg;
}