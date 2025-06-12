/*
 *   ROBOSAMPLE.C -- A sample/template for RoboKar program with uCOS-II
 *   Written by:  Rosbi Mamat  6/5/2014
 *   Updated  :  6/5/2025  Enhanced navigation with continuous recovery, LED blink at A, and refined logic
 *   Updated  :  6/12/2025  Modified to handle track layout with checkpoints A-F and light sensors L1-L2
 */

#include "..\inc\kernel.h"
#include "..\inc\hal_robo.h"

#define STOP_SPEED     0
#define LOW_SPEED     40
#define MEDIUM_SPEED  60
#define HIGH_SPEED    80
#define REVERSE_SPEED -40

#define TASK_STK_SZ            128
#define TASK_START_PRIO          1
#define TASK_CHKCOLLIDE_PRIO     2
#define TASK_CTRLMOTOR_PRIO      3
#define TASK_NAVIG_PRIO          4

OS_STK TaskStartStk[TASK_STK_SZ];
OS_STK ChkCollideStk[TASK_STK_SZ];
OS_STK CtrlmotorStk[TASK_STK_SZ];
OS_STK NavigStk[TASK_STK_SZ];

struct robostate
{
    int rspeed;
    int lspeed;
    char obstacle;
    int score;
} myrobot;

typedef enum { CP_START, CP_A, CP_B, CP_C, CP_D, CP_E, CP_F, CP_DONE } CpState;
static CpState  cp_state = CP_START;
static char     seenL1   = 0;
static char     seenL2   = 0;
static char     performedL2Task = 0;
static int      lightThreshold = 200;  // Threshold for light sensor detection

void blinkLED(char times, int interval_ms);

void CheckCollision(void *data)
{
    for (;;)
    {
        myrobot.obstacle = (robo_proxSensor() == 1);
        OSTimeDlyHMSM(0, 0, 0, 100);
    }
}

void CntrlMotors(void *data)
{
    int speed_r, speed_l;
    for (;;)
    {
        speed_r = myrobot.rspeed;
        speed_l = myrobot.lspeed;
        robo_motorSpeed(speed_l, speed_r);
        OSTimeDlyHMSM(0, 0, 0, 100);
    }
}

void Navig(void *data)
{
    for (;;)
    {
        int  code     = robo_lineSensor();
        int  lightVal = robo_lightSensor();
        char prox     = robo_proxSensor();

        // Line following logic
        switch (code)
        {
            case 0: // All sensors off track - lost
                // Lost: reverse slightly, then continue
                myrobot.lspeed = REVERSE_SPEED;
                myrobot.rspeed = REVERSE_SPEED;
                break;
            case 1: // Right sensor on track
                myrobot.lspeed = MEDIUM_SPEED;
                myrobot.rspeed = LOW_SPEED/2;
                break;
            case 2: // Middle sensor on track
                myrobot.lspeed = MEDIUM_SPEED;
                myrobot.rspeed = MEDIUM_SPEED;
                break;
            case 3: // Middle and right sensors on track
                myrobot.lspeed = MEDIUM_SPEED;
                myrobot.rspeed = LOW_SPEED;
                break;
            case 4: // Left sensor on track
                myrobot.lspeed = LOW_SPEED/2;
                myrobot.rspeed = MEDIUM_SPEED;
                break;
            case 6: // Left and middle sensors on track
                myrobot.lspeed = LOW_SPEED;
                myrobot.rspeed = MEDIUM_SPEED;
                break;
            case 7: // All sensors on track - full bar
                // Detected full bar: pause briefly then continue
                myrobot.lspeed = STOP_SPEED;
                myrobot.rspeed = STOP_SPEED;
                OSTimeDlyHMSM(0, 0, 0, 200); // Pause at the bar
                myrobot.lspeed = MEDIUM_SPEED;
                myrobot.rspeed = MEDIUM_SPEED;
                break;
            default:
                myrobot.lspeed = LOW_SPEED;
                myrobot.rspeed = LOW_SPEED;
                break;
        }
        
        // Light sensor detection
        if (lightVal > lightThreshold) {
            // Determine which light sensor we're detecting based on checkpoint state
            if (cp_state < CP_C && !seenL1) {
                // Likely detecting L1 (before checkpoint C)
                seenL1 = 1;
                myrobot.score += 5; // Rule 4 - Reaching A without detecting L1 earns 5 points
                robo_Honk(); // Signal detection
            } else if (cp_state >= CP_C && !seenL2) {
                // Likely detecting L2 (after checkpoint C)
                seenL2 = 1;
                robo_Honk(); // Signal detection
                
                // Rule 7.1 - After detecting L2, reverse back to main track
                if (cp_state == CP_D && !performedL2Task) {
                    performedL2Task = 1;
                    
                    // Reverse and turn to get back to main track
                    myrobot.lspeed = REVERSE_SPEED;
                    myrobot.rspeed = REVERSE_SPEED;
                    robo_motorSpeed(myrobot.lspeed, myrobot.rspeed);
                    OSTimeDlyHMSM(0, 0, 1, 0); // Reverse for 1 second
                    
                    // Turn to reorient to main track
                    myrobot.lspeed = MEDIUM_SPEED;
                    myrobot.rspeed = -LOW_SPEED;
                    robo_motorSpeed(myrobot.lspeed, myrobot.rspeed);
                    OSTimeDlyHMSM(0, 0, 1, 500); // Turn for 1.5 seconds
                    
                    myrobot.score += 15; // Additional 15 points for completing L2 task
                }
            }
        }
        
        // Checkpoint detection and scoring
        switch (cp_state)
        {
            case CP_START:
                if (code == 7) { // Full bar at start line
                    cp_state = CP_A;
                }
                break;
            case CP_A:
                if (code == 7) { // Full bar at checkpoint A
                    cp_state = CP_B;
                    myrobot.score += 5; // Rule 5 - Reaching B earns 5 points
                    
                    // Rule 4.1 - Detecting light L1 and making LED blink earns 10 points
                    if (seenL1) {
                        blinkLED(3, 150);
                        myrobot.score += 10;
                    }
                }
                break;
            case CP_B:
                if (code == 7) { // Full bar at checkpoint B
                    cp_state = CP_C;
                    myrobot.score += 5; // Rule 6 - Reaching C earns 5 points
                    robo_LED_toggle();
                }
                break;
            case CP_C:
                if (code == 7) { // Full bar at checkpoint C
                    cp_state = CP_D;
                    myrobot.score += 5; // Rule 7 - Reaching D earns 5 points
                    robo_LED_toggle();
                }
                break;
            case CP_D:
                if (code == 7) { // Full bar at checkpoint D
                    cp_state = CP_E;
                    myrobot.score += 5; // Rule 8 - Reaching E earns 5 points
                    robo_LED_toggle();
                }
                break;
            case CP_E:
                if (code == 7) { // Full bar at checkpoint E
                    cp_state = CP_F;
                    myrobot.score += 5; // Rule 9 - Reaching F earns 5 points
                    robo_LED_toggle();
                }
                break;
            case CP_F:
                if (code == 7) { // Full bar at finish line
                    cp_state = CP_DONE;
                    myrobot.score += 5; // Rule 10 - Reaching the end earns 5 points
                    robo_LED_on(); // Keep LED on at finish
                    
                    // Stop the robot at the finish line
                    myrobot.lspeed = STOP_SPEED;
                    myrobot.rspeed = STOP_SPEED;
                }
                break;
            case CP_DONE:
                // Robot has completed the course
                myrobot.lspeed = STOP_SPEED;
                myrobot.rspeed = STOP_SPEED;
                break;
        }

        // Apply motor speeds
        if (myrobot.obstacle) {
            // Stop if obstacle detected
            robo_motorSpeed(STOP_SPEED, STOP_SPEED);
        } else {
            robo_motorSpeed(myrobot.lspeed, myrobot.rspeed);
        }
        
        OSTimeDlyHMSM(0, 0, 0, 100);
    }
}

void blinkLED(char times, int interval_ms)
{
    int i;
    for (i = 0; i < times; i++)
    {
        robo_LED_on();
        OSTimeDlyHMSM(0, 0, 0, interval_ms);
        robo_LED_off();
        OSTimeDlyHMSM(0, 0, 0, interval_ms);
    }
}

void TaskStart(void *data)
{
    OS_ticks_init();

    OSTaskCreate(CheckCollision, (void*)0,
                &ChkCollideStk[TASK_STK_SZ-1],
                TASK_CHKCOLLIDE_PRIO);

    OSTaskCreate(CntrlMotors, (void*)0,
                &CtrlmotorStk[TASK_STK_SZ-1],
                TASK_CTRLMOTOR_PRIO);

    OSTaskCreate(Navig, (void*)0,
                &NavigStk[TASK_STK_SZ-1],
                TASK_NAVIG_PRIO);

    for (;;)
    {
        OSTimeDlyHMSM(0, 0, 5, 0);
        robo_LED_toggle();
    }
}

int main(void)
{
    robo_Setup();
    OSInit();

    robo_motorSpeed(STOP_SPEED, STOP_SPEED);
    myrobot.rspeed   = STOP_SPEED;
    myrobot.lspeed   = STOP_SPEED;
    myrobot.obstacle = 0;
    myrobot.score    = 0;

    OSTaskCreate(TaskStart, (void*)0,
                &TaskStartStk[TASK_STK_SZ-1],
                TASK_START_PRIO);

    robo_Honk();
    robo_wait4goPress();
    OSStart();
    for (;;);
}