/************************************************************************************************************************************************
*
*	File Name: main.c
*
*	Project: Power System Window Control
*
*	Author: Team 2
*
*	Description: 	1. Implementation of front passenger door window with both passenger and driver control panels.
*								2. FreeRTOS implementation is a must.
*								3. Implementation of 2 limit switches to limit the window motor from top and bottom limits of the window.
*								4. Obstacle detection implementation is required, no need for current stall sensor, just use a push button to indicate jamming.
*
* System basic features: 	1. Manual open/close function
*													When the power window switch is pushed or pulled
*													continuously, the window opens or closes until the switch is released.
*													2. One touch auto open/close function
*													When the power window switch is pushed or pulled
*													shortly, the window fully opens or closes.
*													3. Window lock function
*													When the window lock switch is turned on, the opening and closing of
*													all windows except the driver’s window is disabled.
*													4. Jam protection function
*													This function automatically stops the power window and moves it
*													downward about 0.5 second if foreign matter gets caught in the
*													window during one touch auto close operation.
*
*************************************************************************************************************************************************/

/************************************************************************************************************************************************

																												LIBRARIES INCLUDES

*************************************************************************************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <FreeRTOS.h>
#include <task.h>
#include "queue.h"
#include "semphr.h"
#include "Port_Config.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"

/************************************************************************************************************************************************

																														VARIABLES DEFINITION
																									"Semaphores, Mutexes, Handlers, Variables"

*************************************************************************************************************************************************/

/* Global Variables */
#define WINDOW_OPEN  		HIGH																											// Saving the state of the open window as 1
#define WINDOW_CLOSED  	LOW																												// Saving the state of the closed window as 0
#define UP							HIGH																											// Saving the UP Direction of window as 1
#define DOWN						LOW																												// Saving the DOWN Direction of window as 0
#define MIDDLE					2
#define STOP						0
#define PU							1
#define PD							2
#define DU							3
#define DD							4

bool driver_elevate_button_state;																									// variable for handling the state of the elevator button from the driver side
bool driver_lower_button_state;																										// variable for handling the state of the lowering button from the driver side
bool passenger_elevate_button_state;																							// variable for handling the state of the elevator button from the Passenger side
bool passenger_lower_button_state;																								// variable for handling the state of the Lowering button from the Passenger side
bool lock_state;																																	// variable for handling the state of the lock switch
bool ir_sensor_state;																															// variable for handling the state of the ir sensor
bool upper_limit_switch_state;																										// variable for handling the state of the upper limit switch
bool lower_limit_switch_state;																										// variable for handling the state of the lower limit switch
int	 window_state = MIDDLE;																												// variable for handling the state of the window window state
bool operation;																																		// variable for handling the operation type
//bool behaviour = LOW;																															// variable for handling the return from the manual control
int last_task = STOP;

/* Handlers, Semaphores, Mutexes */
TaskHandle_t 				xDriverWindowElevateTaskHandle 									= NULL;				// handler of the vDriverWindowElevateTask
TaskHandle_t 				xDriverWindowLowerTaskHandle 										= NULL;				// handler of the vDriverWindowLowerTask
TaskHandle_t 				xPassengerWindowElevateTaskHandle 							= NULL;				// handler of the vPassengerWindowElevateTask
TaskHandle_t 				xPassengerWindowLowerTaskHandle 								= NULL;				// handler of the vPassengerWindowLowerTask
TaskHandle_t 				xObsatcleDetectionHandle 												= NULL;				// handler of the vObstacleDetection
TaskHandle_t 				xLockWindowsTaskHandle		 											= NULL;				// handler of the vLockWindowsTask
TaskHandle_t				xUpperLimitActionHandle													= NULL;				// handler of the UpperLimitAction
TaskHandle_t				xLowerLimitActionHandle													= NULL;				// handler of the LowerLimitAction

SemaphoreHandle_t 	xDriverWindowElevateTaskUnlockerSemaphore 			= NULL;				// semaphore for unlocking the vDriverWindowElevateTask
SemaphoreHandle_t 	xDriverWindowLowerTaskUnlockerSemaphore 				= NULL;				// semaphore for unlocking the vDriverWindowLowerTask
SemaphoreHandle_t 	xPassengerWindowElevateTaskUnlockerSemaphore		= NULL;				// semaphore for unlocking the vPassengerWindowElevateTask
SemaphoreHandle_t 	xPassengerWindowLowerTaskUnlockerSemaphore			= NULL;				// semaphore for unlocking the vPassengerWindowLowerTask
SemaphoreHandle_t 	xLockWindowsTaskUnlockerSemaphore 							= NULL;				// semaphore for unlocking the vObstacleDetection
SemaphoreHandle_t 	xObstacleDetectionUnlockerSemaphore 						= NULL;				// semaphore for unlocking the vLockWindowsTask
SemaphoreHandle_t		xUpperLimitUnlockerSemaphore										=	NULL;				// semaphore for unlocking the UpperLimitAction
SemaphoreHandle_t		xLowerLimitUnlockerSemaphore										=	NULL;				// semaphore for unlocking the LowerLimitAction




/************************************************************************************************************************************************

																													 TASKS PROTOTYPE

*************************************************************************************************************************************************/

void vDriverWindowElevateTask			(void *pvParameters);
/*
	RTOS task for controlling (elevating) a passenger window from the driver side
*/
void vDriverWindowLowerTask				(void *pvParameters);
/*
	RTOS task for controlling (lowering) a passenger window from the driver side
*/
void vPassengerWindowElevateTask	(void *pvParameters);
/*
	RTOS task for controlling (elevating) a passenger window from the passenger side
*/
void vPassengerWindowLowerTask		(void *pvParameters);
/*
	RTOS task for controlling (lowering) a passenger window from the passenger side
*/
void vLockWindows									(void *pvParameters);
/*
	RTOS task for locking/unlocking the control from the passenger side
*/
void vObstacleDetection						(void *pvParameters);
/*
	RTOS task for detecting obstacles in the path of the window during automatic closing
*/
void vUpperLimitAction						(void *pvParameters);
/*
	RTOS task for reacting to reaching the upper limit
*/
void vLowerLimitAction						(void *pvParameters);
/*
	RTOS task for reacting to reaching the lower limit
*/
void ISRHandlers									(void);
/*
	Function responsible for handling the interrupts
*/

/************************************************************************************************************************************************

																														MAIN FUNCTION

*************************************************************************************************************************************************/

int main()
{
	// GPIO Initialization
	PortA_Config();																																											// Set up port A for requirements
	PortC_Config();																																											// Set up port C for requirements 
	PortF_Config();																																											// Set up port F for requirements 
	
	// INTERRUPTS INITIALIZATION
	GPIOIntRegister	(GPIO_PORTA_BASE, ISRHandlers);																											// Setting the function ISRHandler as a react to interrupts from PORTA
	GPIOIntRegister	(GPIO_PORTC_BASE, ISRHandlers);																											// Setting the function ISRHandler as a react to interrupts from PORTC
	
	// Setting the interrupt triggering types for PORTA
  GPIOIntTypeSet	(GPIO_PORTA_BASE, Passenger_Elevate_Button, GPIO_RISING_EDGE);
	GPIOIntTypeSet	(GPIO_PORTA_BASE, Driver_Elevate_Button		, GPIO_RISING_EDGE);
	GPIOIntTypeSet	(GPIO_PORTA_BASE, Passenger_Lower_Button	, GPIO_RISING_EDGE);
  GPIOIntTypeSet	(GPIO_PORTA_BASE, Driver_Lower_Button			, GPIO_RISING_EDGE);
	
	// Setting the interrupt triggering types for PORTC
	GPIOIntTypeSet	(GPIO_PORTC_BASE, Object_Detection_Sensor	, GPIO_BOTH_EDGES);
	GPIOIntTypeSet	(GPIO_PORTC_BASE, Window_Lower_Limit			, GPIO_BOTH_EDGES);
	GPIOIntTypeSet	(GPIO_PORTC_BASE, Window_Upper_Limit			, GPIO_BOTH_EDGES);
	GPIOIntTypeSet	(GPIO_PORTC_BASE, Window_Lock_Switch			, GPIO_BOTH_EDGES);
	
	// Set the interrupt priorities for the pins of Port C and Port A
	IntPrioritySet(INT_GPIOC_TM4C123, configMAX_SYSCALL_INTERRUPT_PRIORITY);
	IntPrioritySet(INT_GPIOA_TM4C123, configMAX_SYSCALL_INTERRUPT_PRIORITY+1);
	
	// Enabling the interrupts for PORTA pins
  GPIOIntEnable		(GPIO_PORTA_BASE, GPIO_INT_PIN_2);
  GPIOIntEnable		(GPIO_PORTA_BASE, GPIO_INT_PIN_3);
  GPIOIntEnable		(GPIO_PORTA_BASE, GPIO_INT_PIN_4);
	GPIOIntEnable		(GPIO_PORTA_BASE, GPIO_INT_PIN_5);
	
	// Enabling the interrupts for PORTC pins
	GPIOIntEnable		(GPIO_PORTC_BASE, GPIO_INT_PIN_4);
  GPIOIntEnable		(GPIO_PORTC_BASE, GPIO_INT_PIN_5);
	GPIOIntEnable		(GPIO_PORTC_BASE, GPIO_INT_PIN_6);
  GPIOIntEnable		(GPIO_PORTC_BASE, GPIO_INT_PIN_7);
	
	/* Create tasks, semaphores, and queues */
	xDriverWindowElevateTaskUnlockerSemaphore			= xSemaphoreCreateBinary();
	xDriverWindowLowerTaskUnlockerSemaphore				= xSemaphoreCreateBinary();
	xPassengerWindowElevateTaskUnlockerSemaphore	= xSemaphoreCreateBinary();
	xPassengerWindowLowerTaskUnlockerSemaphore		= xSemaphoreCreateBinary();
	xLockWindowsTaskUnlockerSemaphore							= xSemaphoreCreateBinary();
	xObstacleDetectionUnlockerSemaphore						= xSemaphoreCreateBinary();
	xLowerLimitUnlockerSemaphore									= xSemaphoreCreateBinary();
	xUpperLimitUnlockerSemaphore									= xSemaphoreCreateBinary();

	/* Tasks Creation */
	xTaskCreate(vPassengerWindowElevateTask		, "PassengerWindowElevate"	, 150, NULL, 1, &xPassengerWindowElevateTaskHandle);
	xTaskCreate(vPassengerWindowLowerTask			, "PassengerWindowLower"		, 150, NULL, 1, &xPassengerWindowLowerTaskHandle);
	xTaskCreate(vDriverWindowElevateTask			, "DriverWindowElevate"			, 150, NULL, 1, &xDriverWindowElevateTaskHandle);
	xTaskCreate(vDriverWindowLowerTask				, "DriverWindowLower"				, 150, NULL, 1, &xDriverWindowLowerTaskHandle);
	xTaskCreate(vLockWindows									, "LockWindows"							, 100, NULL, 2, &xLockWindowsTaskHandle);
	xTaskCreate(vUpperLimitAction							, "UpperLimitAction"				, 100, NULL, 3, xUpperLimitActionHandle);
	xTaskCreate(vLowerLimitAction							, "LowerLimitAction"				, 100, NULL, 3, xUpperLimitActionHandle);
	xTaskCreate(vObstacleDetection						, "ObstacleDetection"				, 100, NULL, 4, xObsatcleDetectionHandle);

	/* Start the FreeRTOS scheduler */
	vTaskStartScheduler();

	while (1) 
		{
			/* Should not reach here */
		}
}

/************************************************************************************************************************************************

																													TASKS DEFINITION

*************************************************************************************************************************************************/

/* 
	Driver window Elevate task
	it handles both the automatic and manual elevation modes from the driver side
*/
void vDriverWindowElevateTask(void *pvParameters) {
	// taking semaphore at the beginning for blocking the task
	xSemaphoreTake(xDriverWindowElevateTaskUnlockerSemaphore, 0);
	while (1){
		xSemaphoreTake(xDriverWindowElevateTaskUnlockerSemaphore, portMAX_DELAY);
		// this point will be reached only when interrupt occurs from the driver's elevator pushbutton
		// delay for debounce
		vTaskDelay(500/portTICK_RATE_MS);
		
		driver_elevate_button_state = GPIOPinRead(Buttons_Motor_Port, Driver_Elevate_Button);
		// MANUAL MODE
		if ( driver_elevate_button_state == HIGH && window_state != WINDOW_CLOSED){
			// as long as the button is pressed
			while (driver_elevate_button_state == HIGH && window_state != WINDOW_CLOSED){
				// move window up
				GPIO_PORTA_DATA_R |= DC_Motor_In1;
				GPIO_PORTA_DATA_R &= ~DC_Motor_In2;
				operation = UP;
				window_state = MIDDLE;
				driver_elevate_button_state = GPIOPinRead(Buttons_Motor_Port, Driver_Elevate_Button);
				last_task = DU;
			}
			GPIO_PORTA_DATA_R &= ~DC_Motor_In1;
			GPIO_PORTA_DATA_R &= ~DC_Motor_In2;
			last_task = STOP;
		}
		// AUTOMATIC MODE
		else if (driver_elevate_button_state == LOW && window_state != WINDOW_CLOSED){
			// move window up
			GPIO_PORTA_DATA_R |= DC_Motor_In1;
			GPIO_PORTA_DATA_R &= ~DC_Motor_In2;
			operation = UP;
			window_state = MIDDLE;
			last_task = DU;
		}
	}
}

/* 
	Driver window Lower task
	it handles both the automatic and manual Lowering modes from the driver side
*/
void vDriverWindowLowerTask(void *pvParameters) {
	// taking semaphore at the beginning for blocking the task
	xSemaphoreTake(xDriverWindowLowerTaskUnlockerSemaphore, 0);
	while (1){
		xSemaphoreTake(xDriverWindowLowerTaskUnlockerSemaphore, portMAX_DELAY);
		// this point will be reached only when interrupt occurs from the driver's Lower pushbutton
		// delay for debounce
		vTaskDelay(500/portTICK_RATE_MS);
		
		driver_lower_button_state = GPIOPinRead(Buttons_Motor_Port, Driver_Lower_Button);
		// MANUAL MODE
		if ( driver_lower_button_state == HIGH && window_state != WINDOW_OPEN){
			while (driver_lower_button_state == HIGH && window_state != WINDOW_OPEN){
				// move window down
				GPIO_PORTA_DATA_R &= ~DC_Motor_In1;
				GPIO_PORTA_DATA_R |= DC_Motor_In2;
				operation = DOWN;
				window_state = MIDDLE;
				last_task = DD;
				driver_lower_button_state = GPIOPinRead(Buttons_Motor_Port, Driver_Lower_Button);
			}
			GPIO_PORTA_DATA_R &= ~DC_Motor_In1;
			GPIO_PORTA_DATA_R &= ~DC_Motor_In2;
			last_task = STOP;
		}
		// AUTOMATIC MODE
		else if (driver_elevate_button_state == LOW && window_state != WINDOW_OPEN){
			// move window down
			GPIO_PORTA_DATA_R &= ~DC_Motor_In1;
			GPIO_PORTA_DATA_R |= DC_Motor_In2;
			operation = DOWN;
			window_state = MIDDLE;
			last_task = DD;
		}
	}
}

/* 
	Passenger window Elevate task
	it handles both the automatic and manual elevation modes from the passenger side
*/
void vPassengerWindowElevateTask(void *pvParameters) {
	// taking semaphore at the beginning for blocking the task
	xSemaphoreTake(xPassengerWindowElevateTaskUnlockerSemaphore, 0);
	while (1){
		xSemaphoreTake(xPassengerWindowElevateTaskUnlockerSemaphore, portMAX_DELAY);
		// this point will be reached only when interrupt occurs from the Passenger's upper pushbutton
		// delay for debounce
		vTaskDelay(500/portTICK_RATE_MS);
		
		passenger_elevate_button_state = GPIOPinRead(Buttons_Motor_Port, Passenger_Elevate_Button);
		// MANUAL MODE
		if ( passenger_elevate_button_state == HIGH && window_state != WINDOW_CLOSED){
			while (passenger_elevate_button_state == HIGH && window_state != WINDOW_CLOSED){
				// move window up
				GPIO_PORTA_DATA_R |= DC_Motor_In1;
				GPIO_PORTA_DATA_R &= ~DC_Motor_In2;
				last_task = PU;
				operation = UP;
				window_state = MIDDLE;
				passenger_elevate_button_state = GPIOPinRead(Buttons_Motor_Port, Passenger_Elevate_Button);
			}
			GPIO_PORTA_DATA_R &= ~DC_Motor_In1;
			GPIO_PORTA_DATA_R &= ~DC_Motor_In2;
			last_task = STOP;
		}
		// AUTOMATIC MODE
		else if (passenger_elevate_button_state == LOW && window_state != WINDOW_CLOSED){
			// move window up
			GPIO_PORTA_DATA_R |= DC_Motor_In1;
			GPIO_PORTA_DATA_R &= ~DC_Motor_In2;
			operation = UP;
			last_task = PU;
			window_state = MIDDLE;
		}
	}
}

/* 
	Passenger window lower task
	it handles both the automatic and manual lowering modes from the passenger side
*/
void vPassengerWindowLowerTask(void *pvParameters) {
	// taking semaphore at the beginning for blocking the task
	xSemaphoreTake(xPassengerWindowLowerTaskUnlockerSemaphore, 0);
	while (1){
		xSemaphoreTake(xPassengerWindowLowerTaskUnlockerSemaphore, portMAX_DELAY);
		// this point will be reached only when interrupt occurs from the passenger's Lower pushbutton
		// delay for debounce
		vTaskDelay(500/portTICK_RATE_MS);
		
		passenger_lower_button_state = GPIOPinRead(Buttons_Motor_Port, Passenger_Lower_Button);
		// MANUAL MODE
		if ( passenger_lower_button_state == HIGH && window_state != WINDOW_OPEN){
			while (passenger_lower_button_state == HIGH && window_state != WINDOW_OPEN){
				// move window down
				GPIO_PORTA_DATA_R &= ~DC_Motor_In1;
				GPIO_PORTA_DATA_R |= DC_Motor_In2;
				operation = DOWN;
				last_task = PD;
				window_state = MIDDLE;
				passenger_lower_button_state = GPIOPinRead(Buttons_Motor_Port, Passenger_Lower_Button);
			}
			GPIO_PORTA_DATA_R &= ~DC_Motor_In1;
			GPIO_PORTA_DATA_R &= ~DC_Motor_In2;
			last_task = STOP;
		}
		// AUTOMATIC MODE
		else if (passenger_elevate_button_state == LOW && window_state != WINDOW_OPEN){
			// move window down
			GPIO_PORTA_DATA_R &= ~DC_Motor_In1;
			GPIO_PORTA_DATA_R |= DC_Motor_In2;
			operation = DOWN;
			last_task = PD;
			window_state = MIDDLE;
		}
	}
}

/* Task to lock/unlock the passenger side control of window */
void vLockWindows(void *pvParameters) {
	// taking semaphore at the beginning for blocking the task
	xSemaphoreTake(xLockWindowsTaskUnlockerSemaphore, 0);
	while (1){
		xSemaphoreTake(xLockWindowsTaskUnlockerSemaphore, portMAX_DELAY);
		// delay for debounce
		vTaskDelay(500/portTICK_RATE_MS);
		
		lock_state = GPIOPinRead(Sensors_Port,Window_Lock_Switch);
		// DISABLING THE PASSENGER CONTROL
		if (lock_state == HIGH){
			vTaskSuspend(xPassengerWindowElevateTaskHandle);
			vTaskSuspend(xPassengerWindowLowerTaskHandle);
			xSemaphoreTake(xLockWindowsTaskUnlockerSemaphore, portMAX_DELAY);
		}
		// ALLOWING PASSENGER CONTROL
		else{
			vTaskResume(xPassengerWindowElevateTaskHandle);
			vTaskResume(xPassengerWindowLowerTaskHandle);
			xSemaphoreTake(xLockWindowsTaskUnlockerSemaphore, portMAX_DELAY);
		}
	}
}

/* Task for obstacle detection */
void vObstacleDetection(void *pvParameters) {
	// taking semaphore at the beginning for blocking the task
	xSemaphoreTake(xObstacleDetectionUnlockerSemaphore, 0);
	while (1) {
		xSemaphoreTake(xObstacleDetectionUnlockerSemaphore, portMAX_DELAY);
		vTaskDelay(200/portTICK_RATE_MS);
		ir_sensor_state = GPIOPinRead(Sensors_Port,Object_Detection_Sensor);
		// PROHIBITING WINDOW MOVEMENT 
		if ( ir_sensor_state == LOW && operation == UP) {  // Function to check if an obstacle is detected
			GPIO_PORTA_DATA_R &= ~DC_Motor_In1;
			GPIO_PORTA_DATA_R &= ~DC_Motor_In2;
			
			vTaskDelay(1000/portTICK_RATE_MS);
			
			GPIO_PORTA_DATA_R &= ~DC_Motor_In1;
			GPIO_PORTA_DATA_R |= DC_Motor_In2;
			
			vTaskDelay(1000/portTICK_RATE_MS);
			
			GPIO_PORTA_DATA_R &= ~DC_Motor_In1;
			GPIO_PORTA_DATA_R &= ~DC_Motor_In2;
			vTaskSuspend(xDriverWindowElevateTaskHandle);
			vTaskSuspend(xPassengerWindowElevateTaskHandle);
		}
		// RE-ALLOW MOVEMENT OF WINDOW
		else {
			vTaskResume(xDriverWindowElevateTaskHandle);
			vTaskResume(xPassengerWindowElevateTaskHandle);
		}
	}
}

/* Task for Upper Limit detection */
void vUpperLimitAction(void *pvParameters) {
	// taking semaphore at the beginning for blocking the task
	xSemaphoreTake(xUpperLimitUnlockerSemaphore, 0);
	while (1) {
		xSemaphoreTake(xUpperLimitUnlockerSemaphore, portMAX_DELAY);
		upper_limit_switch_state = GPIOPinRead(Sensors_Port, Window_Upper_Limit);
		// STOPPING MOTOR AT UPPER LIMIT
		if ( (upper_limit_switch_state == LOW) && (operation == UP)) {  // Function to check if an obstacle is detected
			GPIO_PORTA_DATA_R &= ~DC_Motor_In1;
			GPIO_PORTA_DATA_R &= ~DC_Motor_In2;
			window_state = WINDOW_CLOSED;
			vTaskSuspend(xDriverWindowElevateTaskHandle);
			vTaskSuspend(xPassengerWindowElevateTaskHandle);
		}
		// RE-ALLOW WINDOW CLOSING OPTION
		else {
			vTaskResume(xDriverWindowElevateTaskHandle);
			vTaskResume(xPassengerWindowElevateTaskHandle);
		}
	}
}

/* Task for Lower Limit detection */
void vLowerLimitAction(void *pvParameters) {
	// taking semaphore at the beginning for blocking the task
	xSemaphoreTake(xLowerLimitUnlockerSemaphore, 0);
	while (1) {
		xSemaphoreTake(xLowerLimitUnlockerSemaphore, portMAX_DELAY);
		lower_limit_switch_state = GPIOPinRead(Sensors_Port, Window_Lower_Limit);
		// STOPPING MOTOR AT LOWER LIMIT
		if ( (lower_limit_switch_state == LOW) && (operation == DOWN) ) {  // Function to check if an obstacle is detected
			GPIO_PORTA_DATA_R &= ~DC_Motor_In1;
			GPIO_PORTA_DATA_R &= ~DC_Motor_In2;
			window_state = WINDOW_OPEN;
			vTaskSuspend(xDriverWindowLowerTaskHandle);
			vTaskSuspend(xPassengerWindowLowerTaskHandle);
		}
		// RE-ALLOWING WINDOW OPENING OPTION
		else {
			vTaskResume(xDriverWindowLowerTaskHandle);
			vTaskResume(xPassengerWindowLowerTaskHandle);
		}
	}
}

/* Idle Task for sleeping the processor */
void vApplicationIdleHook( void ) {
	// checking for intial condition of switch
	lock_state = GPIOPinRead(Sensors_Port,Window_Lock_Switch);
	// DISABLING THE PASSENGER CONTROL
	if (lock_state == HIGH){
		vTaskSuspend(xPassengerWindowElevateTaskHandle);
		vTaskSuspend(xPassengerWindowLowerTaskHandle);
	}
	/* Put the microcontroller in a low power mode */
  SysCtlSleep();
}

/************************************************************************************************************************************************

																													FUNCTIONS DEFINITION

*************************************************************************************************************************************************/

// Function for interrupts handling
void ISRHandlers()
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	
	// If interrupt came from Passenger_Elevate_Button
  // CLEAR INTERRUPT FLAG
  if (GPIOIntStatus(GPIO_PORTA_BASE, Passenger_Elevate_Button) == Passenger_Elevate_Button)
  {
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_INT_PIN_2);
		lock_state = GPIOPinRead(Sensors_Port,Window_Lock_Switch);
		// DISABLING THE PASSENGER CONTROL
		if (lock_state == HIGH){
			
		}
		else if(lock_state == LOW && last_task == STOP){
		// Give the semaphore
    xSemaphoreGiveFromISR(xPassengerWindowElevateTaskUnlockerSemaphore, &xHigherPriorityTaskWoken);

    // If giving the semaphore unblocked a higher-priority task, yield to it
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
		else if(lock_state == LOW && (last_task == PU || last_task == DU || last_task == DD || last_task == PD)){
			GPIO_PORTA_DATA_R &= ~DC_Motor_In1;
			GPIO_PORTA_DATA_R &= ~DC_Motor_In2;
			last_task = STOP;
		}
  }
	
	
	// If interrupt came from Passenger_Lower_Button
  // CLEAR INTERRUPT FLAG
  else if (GPIOIntStatus(GPIO_PORTA_BASE, Passenger_Lower_Button) == Passenger_Lower_Button)
  {
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_INT_PIN_3);
		lock_state = GPIOPinRead(Sensors_Port,Window_Lock_Switch);
		// DISABLING THE PASSENGER CONTROL
		if (lock_state == HIGH){
			
		}
		else if(lock_state == LOW && last_task == STOP){
		// Give the semaphore
    xSemaphoreGiveFromISR(xPassengerWindowLowerTaskUnlockerSemaphore, &xHigherPriorityTaskWoken);

    // If giving the semaphore unblocked a higher-priority task, yield to it
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
		else if(lock_state == LOW && (last_task == PU || last_task == DU || last_task == DD || last_task == PD)){
			GPIO_PORTA_DATA_R &= ~DC_Motor_In1;
			GPIO_PORTA_DATA_R &= ~DC_Motor_In2;
			last_task = STOP;
		}
  }
	
	
  // If interrupt came from Driver_Elevate_Button
  // CLEAR INTERRUPT FLAG
  else if (GPIOIntStatus(GPIO_PORTA_BASE, Driver_Elevate_Button) == Driver_Elevate_Button)
  {
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_INT_PIN_4);
		
		if(last_task == STOP){
		// Give the semaphore
    xSemaphoreGiveFromISR(xDriverWindowElevateTaskUnlockerSemaphore, &xHigherPriorityTaskWoken);

    // If giving the semaphore unblocked a higher-priority task, yield to it
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
		else if(last_task == PU || last_task == DU || last_task == DD || last_task == PD){
			GPIO_PORTA_DATA_R &= ~DC_Motor_In1;
			GPIO_PORTA_DATA_R &= ~DC_Motor_In2;
			last_task = STOP;
		}
  }
	
	
	// If interrupt came from Driver_Lower_Button
  // CLEAR INTERRUPT FLAG
  else if (GPIOIntStatus(GPIO_PORTA_BASE, Driver_Lower_Button) == Driver_Lower_Button)
  {
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_INT_PIN_5);
		
		if(last_task == STOP){
		// Give the semaphore
    xSemaphoreGiveFromISR(xDriverWindowLowerTaskUnlockerSemaphore, &xHigherPriorityTaskWoken);

    // If giving the semaphore unblocked a higher-priority task, yield to it
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
		else if(last_task == PU || last_task == DU || last_task == DD || last_task == PD){
			GPIO_PORTA_DATA_R &= ~DC_Motor_In1;
			GPIO_PORTA_DATA_R &= ~DC_Motor_In2;
			last_task = STOP;
		}
  }
	
	
	// If interrupt came from passing object
  // CLEAR INTERRUPT FLAG
  else if (GPIOIntStatus(GPIO_PORTC_BASE, Object_Detection_Sensor) == Object_Detection_Sensor)
  {
    GPIOIntClear(GPIO_PORTC_BASE, GPIO_INT_PIN_4);
		
		// Give the semaphore
    xSemaphoreGiveFromISR(xObstacleDetectionUnlockerSemaphore, &xHigherPriorityTaskWoken);

    // If giving the semaphore unblocked a higher-priority task, yield to it
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
	
	
	// If interrupt came from Lower Limit
  // CLEAR INTERRUPT FLAG
  else if (GPIOIntStatus(GPIO_PORTC_BASE, Window_Lower_Limit) == Window_Lower_Limit)
  {
    GPIOIntClear(GPIO_PORTC_BASE, GPIO_INT_PIN_6);
		
		// Give the semaphore
    xSemaphoreGiveFromISR(xLowerLimitUnlockerSemaphore, &xHigherPriorityTaskWoken);

    // If giving the semaphore unblocked a higher-priority task, yield to it
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
	
	
	// If interrupt came from Upper Limit
  // CLEAR INTERRUPT FLAG
  else if (GPIOIntStatus(GPIO_PORTC_BASE, Window_Upper_Limit) == Window_Upper_Limit)
  {
    GPIOIntClear(GPIO_PORTC_BASE, GPIO_INT_PIN_5);
		
		// Give the semaphore
    xSemaphoreGiveFromISR(xUpperLimitUnlockerSemaphore, &xHigherPriorityTaskWoken);

    // If giving the semaphore unblocked a higher-priority task, yield to it
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
	
	
	// If interrupt came from lock switch
  // CLEAR INTERRUPT FLAG
  else if (GPIOIntStatus(GPIO_PORTC_BASE, Window_Lock_Switch) == Window_Lock_Switch)
  {
    GPIOIntClear(GPIO_PORTC_BASE, GPIO_INT_PIN_7);
		
		// Give the semaphore
    xSemaphoreGiveFromISR(xLockWindowsTaskUnlockerSemaphore, &xHigherPriorityTaskWoken);

    // If giving the semaphore unblocked a higher-priority task, yield to it
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}
