#include <project.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Motor.h"
#include "Ultra.h"
#include "Nunchuk.h"
#include "Reflectance.h"
#include "Gyro.h"
#include "Accel_magnet.h"
#include "LSM303D.h"
#include "IR.h"
#include "Beep.h"
#include "mqtt_sender.h"
#include <time.h>
#include <sys/time.h>
#include "serial1.h"
#include <unistd.h>

enum direction{forward, right, back, left};

void robot_turn_left(int speed, int delay); 
void robot_turn_right(int speed, int delay); 
void robot_keep_on_line(struct sensors_ dig);
void switch_dir(enum direction *dir, char new_dir);


//Zumo
#if 0
void zmain(void){
    struct sensors_ dig;
    IR_Start();
    reflectance_start();
    motor_start();
    Ultra_Start();
    reflectance_set_threshold(9000, 9000, 11000, 11000, 9000, 9000); // set center sensor threshold to 11000 and others to 9000
    
    int angle = 0;
    int start_time = 0;
    
    while(true){
        reflectance_digital(&dig);
        //Got to the starting line. Wait until IR
        if(dig.l3 && dig.r3){
            motor_forward(0,0);
            IR_wait();
            print_mqtt("Zumo011/ready", "zumo");
            //move over the starting line
            while(dig.l1 || dig.r1 || dig.l3 || dig.r3){
                reflectance_digital(&dig);
                motor_forward(100,5);
            }
            start_time = xTaskGetTickCount();
            print_mqtt("Zumo011/start", "%d", start_time);
            break;
        }
        else{
            motor_forward(50,5);
        }
    }
    
    while(true){
        int d = Ultra_GetDistance();
        reflectance_digital(&dig);
        int time = xTaskGetTickCount() * 2;
        //If left sensors see a line turn right
        if(dig.l2 || dig.l3){
            robot_turn_right(255, time % 255);
            angle += 45;
        }
        //If right sensors see a line turn left
        else if(dig.r2 || dig.r3){
            robot_turn_left(255, time % 255);
            angle -= 45;
        }
        
        //turn left if there is an object infront of the robot
        if(d < 10){
            robot_turn_left(255, time % 255);
        }
        else{
            motor_forward(150,5);
        }
        
        //stop the robot when the button is pressed
        if(SW1_Read() == 0){
            motor_forward(0,0);
            int end_time = xTaskGetTickCount();
            print_mqtt("Zumo011/stop", "%d", end_time);
            print_mqtt("Zumo011/time", "%d", end_time - start_time);
            break;
        }
    }
    while(true){
        vTaskDelay(100);
    }
}    


#endif

//Line follow
#if 0
void firstline(void);
void linefollow(void);
void stopthis(void);

    
void zmain(void)
{
    motor_start();              // enable motor controller
    reflectance_start();        // enable reflectance sensor
    reflectance_set_threshold(9000,9000,11000,11000,9000,9000); // reflectance threshold 
    IR_Start();      // enables  infrared sensors
    
    TickType_t start; 
    TickType_t end;
    int finaltime, finaltime_tics, starttime, endtime;
    struct sensors_ dig;
    
    print_mqtt("Zumo011: Fetching starting position...", "Press button while I'm on track.");
    while(1)      // while loop to wait for button press
    {
        vTaskDelay(3);
        if(!SW1_Read())
        {
            vTaskDelay(300);
            break;
        }
    }
    firstline();             // acquires starting position (first black line that crosses track)
    print_mqtt("Zumo011: ready", "line");
    IR_flush();              // clears previous IR sensor data
    IR_wait();               // waits for IR signal to start following line
    start = xTaskGetTickCount();
    starttime = start / 1000;
    print_mqtt("Zumo011: start_tick", "%d", start);
    if(dig.r3 && dig.l3)      // exit starting line
    {
        motor_forward(255,400);
    }
    linefollow();     // linefollow function
    
    
    end = xTaskGetTickCount();
    endtime = end / 1000;
    finaltime = endtime - starttime;
    finaltime_tics = end - start;
    print_mqtt("Zumo011: endt_tick:","%d", end);
    print_mqtt("Zumo011: The final time is:", "%d sec (%d tics)", finaltime, finaltime_tics);
    stopthis();   // stop function
    
}
void firstline()    // drive to starting position
{
    struct sensors_ dig;
    
    while(1)    // while loop to drive forward until starting line is encountered
    {
       reflectance_digital(&dig);
       if(dig.r1 && dig.l1 && !dig.r3 && !dig.l3)    // middle sensors true but peripheral sensors false: go forward
       {
          motor_forward(20,2);
       }
       if(dig.r3 && dig.l3)    // peripheral sensors true: stop and break while loop
       {
          motor_forward(0,0);
          break;
       }
    }
    vTaskDelay (3);
    return;
}
void linefollow()    // linefollowing function
{
    struct sensors_ dig;
    while(1)
    {
    reflectance_digital(&dig);
    if(dig.r1 && dig.l1 && !dig.l3 && !dig.r3)      // if middle sensors read true, go forward
    {
       motor_forward(255,1);
    }
    else if(!dig.r1 && dig.l1)    // if statement for turning
    {
        motor_turn(0,255,1);
    }
    else if(dig.r1 && !dig.l1)    // if statement for turning
    {
        motor_turn(255,0,1);
    }
    else if(dig.l1 && dig.l3 && dig.r1 && dig.r3)    // encountering first line starts ending sequence
    {
        motor_forward(100,200);    // getting over first line
        while(1)    // while loop that breaks when encountering second line
        {
            motor_forward(100,1);
            reflectance_digital(&dig);
            if(dig.l3 && dig.r3)
            {
                motor_stop();
                vTaskDelay(300);
                return;
            }
            
        }
    }
    }

}
void stopthis()    // function to end main in an infinite while loop
{
    
    while(1)
    {
        motor_stop();    // motor is stopped
        vTaskDelay(100);    // endlessly cycles in vTaskDelay
    }
}
#endif
//Maze
#if 1

void zmain(void)
{
    struct sensors_ dig;
    
    IR_Start();
    IR_flush();
    reflectance_start();
    motor_start();
    Ultra_Start();
    
    reflectance_set_threshold(9000, 8000, 11000, 11000, 8000, 9000); // set center sensor threshold to 11000 and others to 9000
    
    int b_crossed = 0;
    //count coordinates only if first line is crossed so the coordinates will be acureate
    int b_first_line = 0;
    int start_time = 0;
    int end_time = 0;
    
    //Go to first line and wait for IR signal
    while(true){
        reflectance_digital(&dig);
        //go forward to the first line and stop on it
        if(dig.l3 && dig.r3){
            
            motor_forward(0,0);
            print_mqtt("Zumo011/ready", "maze");
            IR_wait();
            //start the clock
            start_time = xTaskGetTickCount();
            print_mqtt("Zumo011/start", "%d", start_time);
            while(dig.l1 && dig.r1 && dig.l3 && dig.r3){
                motor_forward(100,5);
                reflectance_digital(&dig);
            }
            break;
        }
        else{
            motor_forward(50,5);
        }
    }
    
    //coordiante 0 is horizontal line and 1 is vertical line
    int coordinates[] = {0,0};
    enum direction dir = forward;
    
    
    int forward_speed = 255;
    int delay = 3;
    while(true)
    {
        reflectance_digital(&dig); 
        
        //Got to first intersection so turn to left to get to the far left of grid
        if(coordinates[1] == 0 && dig.l3&& dig.l2 && dig.l1 && dig.r1&& dig.r2 && dig.r3 && !b_first_line){
            b_first_line = 1;
            switch_dir(&dir,'l');
            robot_turn_left(100,10);
            motor_forward(100,150);
            continue;
        }
        //if no sersor can see black line back down untill some sensor sees black
        else if(!dig.l3&& !dig.l2 && !dig.l1 && !dig.r1&& !dig.r2 && !dig.r3 ){
            motor_backward(255,25);
        }
        else{
            motor_forward(forward_speed, delay);
        }
        
        
        if(coordinates[1] == 0 && dir == left){
            //at the far left of grid so turn right to face forward and continue the maze
            if(coordinates[0] == -3 && dig.r1&& dig.r2 && dig.r3){
                switch_dir(&dir,'r');
                robot_turn_right(100,50);
                dir = forward;
            }
            else if(dig.r1&& dig.r2 && dig.r3){
                motor_forward(forward_speed, delay);
            }
        }
        
        
        int d = Ultra_GetDistance();
        //Sees an obsticle infront of it. Waits till intersection to avoic the obsitcle
        if((d < 15 && dig.r1&& dig.r2 && dig.r3) ||(d < 15 && dig.l1&& dig.l2 && dig.l3)){
            //If robot is more left than right try right side to avoid the obsticle
            if(coordinates[0] <= 0){
                switch_dir(&dir,'r');
                robot_turn_right(100,50);
            }
            //else robot must be at the middle lane or at right side of grid
            else{
                switch_dir(&dir,'l');
                robot_turn_left(100,50);
            }
            //get the coordinate that the avoidance started
            int start_cord = coordinates[0];
            //On this loop till finds a way to avoid the obsticle
            while(true){
                reflectance_digital(&dig);
                //for every intersection turn and see if the lane is clear to go forward
                if(dig.l3&& dig.l2 && dig.l1 && dig.r1&& dig.r2 && dig.r3){
                    
                    if(start_cord <= 0){
                        coordinates[0]++;
                        switch_dir(&dir,'l');
                        robot_turn_left(100,50);
                    }
                    else{
                        coordinates[0]--;
                        switch_dir(&dir,'r');
                        robot_turn_right(100,50);
                    }
                    motor_forward(0,0);
                    vTaskDelay(30);
                    print_mqtt("Zumo011/position", "%d %d", coordinates[0], coordinates[1]);
                    //Look if there is an obsticle close at the lane
                    d = Ultra_GetDistance();
                    if(d > 15){
                        break;
                    }
                    else{
                        if(start_cord <= 0){
                            robot_turn_right(200,100);
                            robot_turn_right(100,50);
                            d = Ultra_GetDistance();
                            dir = right;
                        }
                        else{
                            robot_turn_left(200,100);
                            robot_turn_left(100,50);
                            d = Ultra_GetDistance();
                            dir = left;
                        }
                    }
                }
                motor_forward(forward_speed, delay);
                robot_keep_on_line(dig);
                
            }
            
        }
        if(dig.r1 && dig.l1 && !dig.l2 && !dig.r2 && b_crossed){
            b_crossed = 0;
        }
        
        //If robot is in the last full line go to the mid line.
        if(coordinates[1] == 11){
            if(coordinates[0] < 0){
                switch_dir(&dir,'r');
            }
            else if(coordinates[0] > 0){
                switch_dir(&dir,'l');
            }
            //robot is in this loop until it has got to the end
            while(true){
                //lower the speed so snesor can see more accuratly the last lines
                motor_forward(100, 1);
                reflectance_digital(&dig);
                robot_keep_on_line(dig);
                //check if robot is at the middle line
                if(coordinates[0] == 0){
                    if(dir == right){
                        switch_dir(&dir,'l');
                    }
                    else if(dir == left){
                        switch_dir(&dir,'r');
                    }
                    break;
                }
                reflectance_digital(&dig);
                if(dig.r1 && dig.l1 && !dig.l2 && !dig.r2 && b_crossed){
                        b_crossed = 0;
                }
                
                if((dig.r1&& dig.r2 && dig.r3 && !b_crossed) || (dig.l1&& dig.l2 && dig.l3 && !b_crossed)){
                    b_crossed = 1;
                    switch(dir){
                        case left:
                            coordinates[0]--;
                            break;
                        case right:
                            coordinates[0]++;
                            break;
                        default:
                            printf("ERROR: no direction");
                            break;
            
                    }
                    print_mqtt("Zumo011/position", "%d %d", coordinates[0], coordinates[1]);
                }

            }
        }
        //Robot is at the end
        else if(coordinates[1] > 11 && !dig.r1&& !dig.l1){
            motor_forward(0,0);
            end_time = xTaskGetTickCount();
            print_mqtt("Zumo011/stop","%d",end_time);
            //printf("at the end of the road. Time took: %.2f\n", (float)(end_time - start_time)/ 1000);
            print_mqtt("Zumo011/time","%d", end_time - start_time);
            break;
        }
  
        
        if((dig.r1&& dig.r2 && dig.r3 && !b_crossed && b_first_line) || (dig.l1&& dig.l2 && dig.l3 && !b_crossed && b_first_line)){
            b_crossed = 1;
            switch(dir){
                case forward:
                    coordinates[1]++;
                    break;
                case left:
                    coordinates[0]--;
                    break;
                case back:
                    coordinates[1]--;
                    break;
                case right:
                    coordinates[0]++;
                    break;
                default:
                    printf("ERROR: no direction");
                    break;
            
            }
            print_mqtt("Zumo011/position", "%d %d", coordinates[0], coordinates[1]);
        }
        
       
        robot_keep_on_line(dig);
    }
    
    while(true){
        vTaskDelay(1000);
    }
}   
#endif

void robot_turn_left(int speed, int delay){
    SetMotors(1,0, speed, speed , delay);
}
void robot_turn_right(int speed, int delay){
    SetMotors(0,1, speed, speed , delay);
}

//Try to keep robot always at the middle of line 
void robot_keep_on_line(struct sensors_ dig){
    if(( dig.l2 && !dig.r2 && !dig.r3) ){
        robot_turn_left(25,5);
    }
    else if( (!dig.l2 && dig.r2 && !dig.l3) ){
        robot_turn_right(25,5);
    }
    else if(( dig.l3 && !dig.r2 && !dig.r3) ){
        robot_turn_right(100,5);
    }
    else if( (!dig.l2 && dig.r3 && !dig.l3) ){
        robot_turn_right(100,5);
    }
    else motor_forward(100,10);
}

//called if there is a need to switch direction 90 degrees.
void switch_dir(enum direction *dir, char new_dir){
    motor_forward(150,100);
    //change direction info acroding to where the robot is looking at the moment
    if(new_dir == 'l'){
        robot_turn_left(200,100);
        switch(*dir){
            case forward:
                *dir = left;
                break;
            case left:
                *dir = back;
                break;
            case back:
                *dir = right;
                break;
            case right:
                *dir = forward;
                break;
            default:
                printf("ERROR: Direction information is wrong");
                break;
        } 
    }
    else if(new_dir == 'r'){
        robot_turn_right(200,100);
        switch(*dir){
            case forward:
                *dir = right;
                break;
            case left:
                *dir = forward;
                break;
            case back:
                *dir = left;
                break;
            case right:
                *dir = back;
                break;
            default:
                printf("ERROR: Direction information is wrong");
                break;
        }
    }
}
/* [] END OF FILE */
