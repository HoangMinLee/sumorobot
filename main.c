/* Include libraries with functions that are needed. */
// tape.1
// tape.2 
#include <NewPing.h>

/* Define pinout of Arduino to match physical connections */
//HINT: All of the Arduino pins are NOT 88

#define PWMA 9 // ~PWM needed
#define AIN1 7
#define AIN2 8
#define PWMB 10 // ~PWM needed
#define BIN1 4
#define BIN2 2
#define STBY 13		

#define SPEAKER 88 	//piezo buzzer pin, ~PWM needed
#define FREQ 1000	//frequency of piezo buzzer
#define LED 88		//sumo LED pin

#define U_TRIG 11	//Ultrasonic Trigger pin, ~PWM needed
#define U_ECHO 12	// Ultrasonic Echo pin, ~PWM needed
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonic( U_TRIG, U_ECHO, MAX_DISTANCE);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET && Is_First_Captured==0)
	{/* xoay phải trong vòng ? s*/}
	else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET && Is_First_Captured==0)
	{/* xoay trái trong vòng ? s*/}
	else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET)
	{/* đi thẳng*/}
	else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET && Is_First_Captured==0)
	{/* đi lùi*/}
	else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET)
	{/* xoay phải trong vòng ? s*/}
	else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET)
	{/* xoay trái trong vòng ? s*/}
	else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET)
	{/* ko làm gì vì ko thể xảy ra*/}
}
void setup() {

/* Define all 7 pins as outputs to the TB6612FNG speed controller */
pinMode(PWMA,OUTPUT);
pinMode(AIN1,OUTPUT);
pinMode(AIN2,OUTPUT);
pinMode(PWMB,OUTPUT);
pinMode(BIN1,OUTPUT);
pinMode(BIN2,OUTPUT);
pinMode(STBY,OUTPUT);
Serial.begin (115200);

						// Open serial monitor at 115200 baud to see ultrasonic values
	startUp();

 delay(500);			//pause 500ms before the match starts  
}
 
void loop() {
		
			//ultrasonic
			int distance = sonic.ping_cm();
			Serial.print("ping:");
			Serial.print (distance);
			Serial.print ("cm");
			
			startUp();

			//search for the other sumobot
			if( distance <= 25){
				goForward ();
				  delay(1000);
			}
			
			//attack by spin
			else{
				rotateLeft();
			}				
			
}


/** IMPORTANT COMBO FUNCTIONS
 * Write your important combo functions here!
 * Make sure to write a function to find and another to attack.
 **/
 
//Write a Function to Find opponent Sumo





//Write Function to Attack opponent Sumo





 
/** BUILT-IN FUNCTION DEFINITIONS
 * Use these functions to make your robot perform
 * basic behaviors.
 * 
 * Note that the PWM values sent to both motors are not equal.
 * Due to variations in motor output, it was found that a duty cycle
 * of about 233 on the left motor and 255 on the right motor makes 
 * a sumobot travel in a straight line at full speed.
 **/
 
 void startUp ()
{
  digitalWrite(STBY,HIGH);
}

void shutDown ()
{
  digitalWrite(STBY,LOW);
}
 
void goForward ()
{
  digitalWrite (AIN1,LOW);
  digitalWrite (AIN2,HIGH);
  analogWrite(PWMA,234);
  digitalWrite (BIN1,HIGH);
  digitalWrite (BIN2,LOW);
  analogWrite(PWMB,255);  
}
 
void goBackward ()
{
  digitalWrite (AIN1,LOW);
  digitalWrite (AIN2,HIGH);
  analogWrite(PWMA,233);
  digitalWrite (BIN1,LOW);
  digitalWrite (BIN2,HIGH);
  analogWrite(PWMB,255);  
}
 
void rotateRight ()
{
  digitalWrite (AIN1,HIGH);
  digitalWrite (AIN2,LOW);
  analogWrite(PWMA,255);
  digitalWrite (BIN1,LOW);
  digitalWrite (BIN2,HIGH);
  analogWrite(PWMB,255);  
}
 
void rotateLeft ()
{
  digitalWrite (AIN1,HIGH);
  digitalWrite (AIN2,LOW);
  analogWrite(PWMA,255);
  digitalWrite (BIN1,HIGH);
  digitalWrite (BIN2,LOW);
  analogWrite(PWMB,255);  
}
 
void veerLeft ()
{
  digitalWrite (AIN1,HIGH);
  digitalWrite (AIN2,LOW);
  analogWrite(PWMA,190);
  digitalWrite (BIN1,HIGH);
  digitalWrite (BIN2,LOW);
  analogWrite(PWMB,255);  
}
 
void veerRight ()
{
  digitalWrite (AIN1,HIGH);
  digitalWrite (AIN2,LOW);
  analogWrite(PWMA,255);
  digitalWrite (BIN1,HIGH);
  digitalWrite (BIN2,LOW);
  analogWrite(PWMB,190);  
}
 
void applyBrakes ()
{
  digitalWrite (AIN1,HIGH);
  digitalWrite (AIN2,HIGH);
  analogWrite(PWMA,255);
  digitalWrite (BIN1,HIGH);
  digitalWrite (BIN2,HIGH);
  analogWrite(PWMB,255);  
}

/** ADDITIONAL COMBO FUNCTIONS
 * Write additional Customized Combination functions here!
 * The following calculations may be useful for your custom methods:
 * (1) A 1 pound sumobot using the 6V Pololu130 motors with a fresh 9V 
 * battery may have wheels that spin about 27 revolutions per minute (RPM).
 * <www.pololu.com/product/1117>
 * (2) The Pololu truck wheels have a 36mm diameter, meaning
 * the wheels travel 36*pi mm for each revolution.
 * <www.pololu.com/product/65>
 * (3) For a robot performing a point turn (with one wheel traveling 
 * forward, and the other traveling backwards), the wheels trace out
 * a circle on the ground with a diameter equal to the distance between
 * the wheels.  For the Tamiya Double Gearbox, that is 60mm.
 * <www.pololu.com/product/114>
 * (4) One full robot rotation will see each wheel traveling 60*pi mm, 
 * Each wheel will spin 60*pi/36*pi times (~1.67x) for a robot rotation.
 * Thus, we have 27/1.67 (16.2) robot rotations per minute.
 * Or 16.2*360/60 (972) degrees of rotation per second (~0.972 deg/ms)
 * (5) Remember that calculations consider ideal conditions.  You
 * will need to test your robot out to get desired results.
 **/

//Function to turn around 180 degrees 
void turnAround()
{
  rotateLeft();
  delay(1370);
}
