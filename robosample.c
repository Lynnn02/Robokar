/*
 *   ROBOSAMPLE.C -- A sample/template for RoboKar program with uCOS-II
 *   Written by:  Rosbi Mamat  6/5/2014
 *   Updated  :  6/5/2025  Enhanced navigation with continuous recovery, LED blink at A, and refined logic
 *   Updated  :  6/12/2025  Modified to handle track layout with checkpoints A-F and light sensors L1-L2
 */

#include "..\inc\kernel.h"
#include "..\inc\hal_robo.h"

#define STOP_SPEED     0
#define LOW_SPEED     30
#define MEDIUM_SPEED  50
#define HIGH_SPEED    60
#define REVERSE_SPEED -30

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
    int lightDetected;
} myrobot;

typedef enum { CP_START, CP_A, CP_B, CP_C, CP_D, CP_E, CP_F, CP_DONE } CpState;
static CpState  cp_state = CP_START;
static char     seenL1   = 0;
static char     seenL2   = 0;
static char     performedL2Task = 0;
static int      lightThreshold = 70;

void blinkLED(char times, int interval_ms);

void CheckCollision(void *data)
{
    static int obstacle_recovery_state = 0;
    static int obstacle_timer = 0;
    
    for (;;)
    {
        char current_obstacle = (robo_proxSensor() == 1);
        
        // Detect new obstacle
        if (current_obstacle && !myrobot.obstacle) {
            // New obstacle detected
            myrobot.obstacle = 1;
            obstacle_recovery_state = 0;
            obstacle_timer = 0;
            
            // Stop immediately and honk to signal obstacle detection
            robo_motorSpeed(STOP_SPEED, STOP_SPEED);
            robo_Honk();
        }
        // Handle obstacle recovery
        else if (myrobot.obstacle) {
            if (!current_obstacle) {
                // Obstacle is gone, but we're still in recovery mode
                obstacle_timer++;
                
                switch (obstacle_recovery_state) {
                    case 0: // Initial backup
                        robo_motorSpeed(REVERSE_SPEED, REVERSE_SPEED);
                        if (obstacle_timer > 10) { // Back up for 1 second
                            obstacle_recovery_state = 1;
                            obstacle_timer = 0;
                        }
                        break;
                    case 1: // Turn to find line
                        robo_motorSpeed(LOW_SPEED, -LOW_SPEED);
                        if (obstacle_timer > 15) { // Turn for 1.5 seconds
                            obstacle_recovery_state = 2;
                            obstacle_timer = 0;
                        }
                        break;
                    case 2: // Move forward slowly to find line
                        robo_motorSpeed(LOW_SPEED, LOW_SPEED);
                        if (robo_lineSensor() != 0 || obstacle_timer > 20) {
                            // Line found or timeout
                            myrobot.obstacle = 0; // End obstacle recovery
                            obstacle_recovery_state = 0;
                            obstacle_timer = 0;
                        }
                        break;
                }
            } else {
                // Obstacle still present, stay stopped
                robo_motorSpeed(STOP_SPEED, STOP_SPEED);
            }
        } else {
            // No obstacle
            myrobot.obstacle = 0;
        }
        
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
    static int lost_counter = 0;
    static int recovery_direction = 1; // 1 for right, -1 for left
    static int last_valid_code = 2; // Default to middle sensor
    
    for (;;)
    {
        int  code     = robo_lineSensor();
        int  lightVal = robo_lightSensor();
        char prox     = robo_proxSensor();

        // Remember last valid line position when not lost
        if (code != 0) {
            last_valid_code = code;
            lost_counter = 0;
        }
        
        // Line following logic
        switch (code)
        {
            case 0: // All sensors off track - lost
                // Implement progressive recovery strategy
                lost_counter++;
                
                if (lost_counter < 5) {
                    // First try backing up slightly
                    myrobot.lspeed = REVERSE_SPEED;
                    myrobot.rspeed = REVERSE_SPEED;
                } else if (lost_counter < 15) {
                    // Then try turning in the direction we last saw the line
                    if (last_valid_code == 1 || last_valid_code == 3) {
                        // Line was on the right, turn right
                        myrobot.lspeed = LOW_SPEED;
                        myrobot.rspeed = -LOW_SPEED;
                    } else if (last_valid_code == 4 || last_valid_code == 6) {
                        // Line was on the left, turn left
                        myrobot.lspeed = -LOW_SPEED;
                        myrobot.rspeed = LOW_SPEED;
                    } else {
                        // Try alternating directions in a widening spiral
                        myrobot.lspeed = recovery_direction * MEDIUM_SPEED;
                        myrobot.rspeed = -recovery_direction * MEDIUM_SPEED;
                        
                        if (lost_counter % 5 == 0) {
                            recovery_direction = -recovery_direction; // Switch direction
                        }
                    }
                } else {
                    // If still lost after extended time, move forward a bit and try again
                    myrobot.lspeed = LOW_SPEED;
                    myrobot.rspeed = LOW_SPEED;
                    
                    if (lost_counter > 25) {
                        lost_counter = 0; // Reset counter to start recovery again
                    }
                }
                break;
            case 1: // Right sensor on track
                // Gentle correction when only right sensor detects the line
                myrobot.lspeed = MEDIUM_SPEED;
                myrobot.rspeed = MEDIUM_SPEED * 0.7;
                break;
            case 2: // Middle sensor on track - straight line
                // Equal speeds for smooth straight movement
                myrobot.lspeed = MEDIUM_SPEED;
                myrobot.rspeed = MEDIUM_SPEED;
                break;
            case 3: // Middle and right sensors on track
                // Gentle right turn
                myrobot.lspeed = MEDIUM_SPEED;
                myrobot.rspeed = MEDIUM_SPEED * 0.8;
                break;
            case 4: // Left sensor on track
                // Gentle correction when only left sensor detects the line
                myrobot.lspeed = MEDIUM_SPEED * 0.7;
                myrobot.rspeed = MEDIUM_SPEED;
                break;
            case 6: // Left and middle sensors on track
                // Gentle left turn
                myrobot.lspeed = MEDIUM_SPEED * 0.8;
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
            case 5: // Left and right sensors on track (unusual case)
                // Both outer sensors - go straight but slower
                myrobot.lspeed = LOW_SPEED;
                myrobot.rspeed = LOW_SPEED;
                break;
            default:
                // Default to gentle forward movement
                myrobot.lspeed = LOW_SPEED;
                myrobot.rspeed = LOW_SPEED;
                break;
        }
        
        // Light sensor detection - values between 0-100, >80 is very bright
        if (lightVal > lightThreshold) {
            // Light detected - turn on LED and honk
            robo_LED_on();
            robo_Honk();
            myrobot.lightDetected = 1;
            
            // Determine which light sensor we're detecting based on checkpoint state
            if (cp_state < CP_C && !seenL1) {
                // Likely detecting L1 (before checkpoint C)
                seenL1 = 1;
                myrobot.score += 5; // Rule 4 - Reaching A without detecting L1 earns 5 points
                
                // Blink LED to acknowledge L1 detection
                blinkLED(2, 100);
            } else if (cp_state >= CP_C && !seenL2) {
                // Likely detecting L2 (after checkpoint C)
                seenL2 = 1;
                
                // Rule 7.1 - After detecting L2, reverse back to main track
                if (cp_state == CP_D && !performedL2Task) {
                    performedL2Task = 1;
                    
                    // Signal L2 detection with double honk
                    robo_Honk();
                    OSTimeDlyHMSM(0, 0, 0, 200);
                    robo_Honk();
                    
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
        } else {
            // No bright light detected
            if (myrobot.lightDetected) {
                // Turn off LED if we were previously detecting light
                robo_LED_off();
                myrobot.lightDetected = 0;
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

        // Apply motor speeds - only if not in obstacle recovery mode
        // The CheckCollision task handles motor control during obstacle recovery
        if (myrobot.obstacle) {
            // Obstacle handling is done in CheckCollision task
            // Don't override those motor commands here
        } else {
            robo_motorSpeed(myrobot.lspeed, myrobot.rspeed);
            
            // Add a small delay when performing recovery maneuvers
            if (code == 0 && lost_counter > 0) {
                OSTimeDlyHMSM(0, 0, 0, 50); // Shorter delay during recovery
            }
            
            // Add a small delay between sensor readings to reduce jitter
            if (code == 2) {
                // On straight line, no additional delay needed
            } else if (code == 3 || code == 6) {
                // Small delay for gentle turns
                OSTimeDlyHMSM(0, 0, 0, 20);
            }
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
    myrobot.lightDetected = 0;

    OSTaskCreate(TaskStart, (void*)0,
                &TaskStartStk[TASK_STK_SZ-1],
                TASK_START_PRIO);

    robo_Honk();
    robo_wait4goPress();
    OSStart();
    for (;;);
}