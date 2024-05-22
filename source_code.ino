#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#define bimax Serial

// Initialize the PCA9685 motor driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


/*                                                                                                                                       ^^^^
                                                                                                                                   |*****    *****|
                                                                                                                                   |+ 1+      -2 -|
                                                                                                                                   |- 4+      -3 -|
                                                                                                                                   |- 5+      -6 +|          
                                                                                                                                   ****************           
*/



// Constants_______________________________________________________________________________________________________

const int InitialPosition[7][2]={{0,0},{100,90},{80,90},{80,90},{80,90},{100,90},{100,90}}; // Initial positions of robot legs
const int motors[7][2]={{0,0},{1,7},{2,8},{15,9}, {4,10},{5,11},{6,12}};                    // Motor mappings for each leg
const int leg_open_plus = 80;                                                               // Angle added to leg open position
const int stand_angle_leg = 90;                                                             // Leg angle when robot is standing
const int incline_stand_angle_leg = 120;                                                    // Leg angle when robot is going under
int open_angle_leg = 150;                                                                   // Leg angle when robot leg is open
const int shoulder_front_angle = 30;                                                        // Margin of movement of shoulder when walking front-back
const int shoulder_turn_angle = 60;                                                         // Margin of movement of shoulder when walking left-right
const int corrections_leg_open_floor = 10;                                                  // Correction value for leg open position
const int arms[2]={0,13};                                                                   // Motor mappings for robot arms
const int Initial_position_arms[2] = {30,30};                                               // Initial positions of robot arms
const int speed_arms = 40;                                                                  // Speed of robot arms movement
//****************************************************************************************************************


//Les variables___________________________________________________________________________________________________

int speed = 7;          // The delay inside each for loop that commands shoulders in front-back
int TurnSpeed = 7;      // The delay inside each for loop that commands shoulders in turn
int HomeSpeed = 50;     // The delay inside each for loop that commands shoulders in home
int orientation[7][2]={{0,0},{1, 1},{-1, -1},{-1, -1},{1, 1},{1, -1},{-1, 1}};        //Not all motors have the same orientation.
int CurrentPosition[7][2]={{0,0},{100,90},{80,90},{80,90},{80,90},{100,90},{100,90}}; // Same value as initial (Manually)
int command; int old_command; int leg_angle_tmp;
int current_Pos_arms[2] = {30,30}; 
//****************************************************************************************************************



//_______________________________________________________les fonctions


// This function takes an angle value as input and converts it to a pulse width value
// for the MG995 servo motor.
int angleToPulseWidth_MG995(int angle) 
{
  // Map the input angle value from the range [0, 180] to the range [125, 490].
  int pulseWidth = map(angle, 0, 180, 125, 490); 
  // Return the calculated pulse width value.
  return pulseWidth;
}


// This function takes an angle value as input and converts it to a pulse width value
// for the SG servo motor.
int angleToPulseWidth_SG(int angle) 
{
  // Map the input angle value from the range [0, 180] to the range [130, 530].
  int pulseWidth = map(angle, 0, 180, 130, 530); 
  // Return the calculated pulse width value.
  return pulseWidth;
}


// This function takes an angle value as input and converts it to a pulse width value
// for the MG996R servo motor.
int angleToPulseWidth_MG996R(int angle) 
{
  // Map the input angle value from the range [0, 180] to the range [130, 570].
  int pulseWidth = map(angle, 0, 180, 130, 570); 
  // Return the calculated pulse width value.
  return pulseWidth;
}

// This function takes an angle value, an index value, and a SorL value as inputs,
// and returns the angle value with modifications based on the orientation, index,
// and SorL values.
int sign_angle(int angle, int index, int SorL) 
{ 
  // Check if the orientation at the given index and SorL is -1.
  // If so, subtract the angle value from 180 to reverse the direction.
  if (orientation[index][SorL] == -1) 
  {
    angle = 180 - angle;
  }

  // If the index is 5 and SorL is 0, call the angleToPulseWidth_MG996R function
  // to convert the angle value to pulse width for MG996R servo motor.
  // Otherwise, call the angleToPulseWidth_MG995 function to convert the angle value
  // to pulse width for MG995 servo motor.
  if (index == 5 && SorL == 0)
  {
    angle = angleToPulseWidth_MG996R(angle);
    /*Serial.println(index);
    Serial.println(SorL);
    Serial.println("is MG996R");*/
  }
  else 
  {
    angle = angleToPulseWidth_MG995(angle);
    /*Serial.println(index);
    Serial.println(SorL);
    Serial.println("is MG995");*/
  }

  // Return the modified angle value.
  return angle;
}


// This function initializes the current position of a servo motor based on the initial position
// and the orientation values at the given index and SorL values.
void Command_MG_initial(int i, int j)
{
  // Call the sign_angle function to get the modified angle value based on the initial position,
  // index, and SorL values, and store it in the CurrentPosition array.
  CurrentPosition[i][j] = sign_angle(InitialPosition[i][j], i, j);
  
  // Set the pulse width of the servo motor using the Adafruit_PWMServoDriver library,
  // based on the motor index and the calculated current position value.
  pwm.setPWM(motors[i][j], 0, CurrentPosition[i][j]);
  // The second parameter (0 in this case) specifies the on/off time of the PWM signal in ticks.
  // This value is not used in the Adafruit_PWMServoDriver library and should always be set to 0.
}


// This function moves all the servo motors to their initial home position, which is either
// the default stand angle for leg servos or a custom angle for other servos.
void GoHome(int leg_angle = stand_angle_leg)
{
  // Loop through all the servos (index 1 to 6) for both SorL values (0 and 1).
  for (int i = 1; i <= 6; i++)
  {
    Command_MG_initial(i, 0); // Call Command_MG_initial for SorL 0
    Command_MG_initial(i, 1); // Call Command_MG_initial for SorL 1
  }

  // Add a delay of 700 milliseconds to allow time for the servos to move to their initial position.
  delay(700);
}


void walk_front(int leg_angle = stand_angle_leg)
{
  // Open legs 1, 3, and 5
  for(int i =1;i<=6;i=i+2) 
  {
    // Set servo motor for shoulder joint of leg i to leg_angle + leg_open_plus
    pwm.setPWM(motors[i][1], 0 , sign_angle(leg_angle+leg_open_plus,i,1));
  }

  // Move legs 1, 3, and 5 to the front and legs 2, 4, and 6 to the back, while registering current positions
  for(int _=0; _<shoulder_front_angle; _++)
  {
    for(int j=1; j<6; j=j+2)
    {
      // Update current position of leg j (1, 3, or 5) by adding orientation value
      CurrentPosition[j][0] += orientation[j][0]; 
      // Set servo motor for hip joint of leg j to the updated current position
      pwm.setPWM(motors[j][0], 0 , CurrentPosition[j][0]);
      
      // Update current position of leg j+1 (2, 4, or 6) by subtracting orientation value
      CurrentPosition[j+1][0] -= orientation[j+1][0]; 
      // Set servo motor for hip joint of leg j+1 to the updated current position
      pwm.setPWM(motors[j+1][0], 0 , CurrentPosition[j+1][0]);
    }
    // Delay to control the speed of movement
    delay(speed);
  }

  // Close legs 1, 3, and 5
  for(int i =1;i<=6;i=i+2) 
  {
    // Set servo motor for shoulder joint of leg i to leg_angle
    pwm.setPWM(motors[i][1], 0 , sign_angle(leg_angle,i,1));
  }
  
  // Delay to control the speed of movement
  delay(speed*4);

  // Open legs 2, 4, and 6
  for(int i =2;i<=6;i=i+2) 
  {
    // Set servo motor for shoulder joint of leg i to leg_angle + leg_open_plus
    pwm.setPWM(motors[i][1], 0 , sign_angle(leg_angle+leg_open_plus,i,1));
  }

  // Move legs 1, 3, and 5 to the back and legs 2, 4, and 6 to the front, while registering current positions
  for(int _=0; _<shoulder_front_angle*2; _++)
  {
    for(int j=1; j<6; j=j+2)
    {
      // Update current position of leg j (1, 3, or 5) by subtracting orientation value
      CurrentPosition[j][0] -= orientation[j][0]; 
      // Set servo motor for hip joint of leg j to the updated current position
      pwm.setPWM(motors[j][0], 0 , CurrentPosition[j][0]);
      
      // Update current position of leg j+1 (2, 4, or 6) by adding orientation value
      CurrentPosition[j+1][0] += orientation[j+1][0]; 
      // Set servo motor for hip joint of leg j+1 to the updated current position
      pwm.setPWM(motors[j+1][0], 0 , CurrentPosition[j+1][0]);
    }
    // Delay to control the speed of movement
    delay(speed);
}

    // Close legs 2, 4, and 6
    for(int i =2;i<=6;i=i+2)
    {
    // Set servo motor for shoulder joint of leg i to leg_angle
        pwm.setPWM(motors[i][1], 0 , sign_angle(leg_angle,i,1));
    }

    // Delay to control the speed of movement
    delay(speed*4);

    // Reset current positions of all legs to their stand position
    for(int i=1; i<=6; i++)
    {
        CurrentPosition[i][0] = stand_position[i][0];
        CurrentPosition[i][1] = stand_position[i][1];
        pwm.setPWM(motors[i][0], 0 , CurrentPosition[i][0]);
        pwm.setPWM(motors[i][1], 0 , CurrentPosition[i][1]);
    }
}


void walk_front_low(int leg_angle = 130)
{
  // Step 1: Open leg 1,3,5
  for(int i =1; i <= 6; i = i + 2) // Iterate through legs 1, 3, 5
  {
    // Determine leg angle with optional corrections for leg 3 when on the floor
    int leg_angle_tmp = i == 3 ? leg_angle + corrections_leg_open_floor : leg_angle ;
    // Set servo PWM signal to open the leg
    pwm.setPWM(motors[i][1], 0 , sign_angle(leg_angle_tmp + leg_open_plus, i, 1));
  }

  // Step 2: Move legs 1,3,5 to the front and legs 2,4,6 to the back, while registering current positions
  for(int _ = 0; _ < shoulder_front_angle; _++) // Repeat for the desired angle of movement
  {
    for(int j = 1; j < 6; j = j + 2) // Iterate through legs 1, 3, 5
    {
      // Update current position of leg j and move it to the front
      CurrentPosition[j][0] += orientation[j][0];
      pwm.setPWM(motors[j][0], 0 , CurrentPosition[j][0]);

      // Update current position of leg j+1 and move it to the back
      CurrentPosition[j+1][0] -= orientation[j+1][0];
      pwm.setPWM(motors[j+1][0], 0 , CurrentPosition[j+1][0]);
    }
    delay(speed); // Add delay for smooth movement
  }

  // Step 3: Close leg 1,3,5
  for(int i = 1; i <= 6; i = i + 2) // Iterate through legs 1, 3, 5
  {
    // Determine leg angle with optional corrections for leg 3 when on the floor
    int leg_angle_tmp = i == 3 ? leg_angle + corrections_leg_open_floor : leg_angle ;
    // Set servo PWM signal to close the leg
    pwm.setPWM(motors[i][1], 0 , sign_angle(leg_angle_tmp, i, 1));
  }

  delay(speed * 4); // Add delay for leg to settle

  // Step 4: Open leg 2,4,6
  for(int i = 2; i <= 6; i = i + 2) // Iterate through legs 2, 4, 6
  {
    // Set servo PWM signal to open the leg
    pwm.setPWM(motors[i][1], 0 , sign_angle(leg_angle + leg_open_plus, i, 1));
  }

  // Step 5: Move legs 1,3,5 to the back and legs 2,4,6 to the front, while registering current positions
  for(int _ = 0; _ < shoulder_front_angle * 2; _++) // Repeat for twice the desired angle of movement
  {
    for(int j = 1; j < 6; j = j + 2) // Iterate through legs 1, 3, 5
    {
      // Update current position of leg j and move it to the back
      CurrentPosition[j][0] -= orientation[j][0];
      pwm.setPWM(motors[j][0], 0 , CurrentPosition[j][0]);

     
    // Update current position of leg j+1 and move it to the front
    CurrentPosition[j+1][0] += orientation[j+1][0];
    pwm.setPWM(motors[j+1][0], 0 , CurrentPosition[j+1][0]);
    }
    delay(speed); // Add delay for smooth movement
   }
   
   // Step 6: Close leg 2,4,6
   for(int i = 2; i <= 6; i = i + 2) // Iterate through legs 2, 4, 6
   {
        // Set servo PWM signal to close the leg
        pwm.setPWM(motors[i][1], 0 , sign_angle(leg_angle, i, 1));
   }
   
   delay(speed * 4); // Add delay for leg to settle
   
   // Step 7: Reset leg positions to neutral position
   for(int i = 1; i <= 6; i++) // Iterate through all legs
   {
        // Set servo PWM signal to neutral position
        pwm.setPWM(motors[i][0], 0, neutral_position[i]);
        pwm.setPWM(motors[i][1], 0, neutral_position[i+6]);
   }
}


//No need to add detailed comments, it simular to walk front
void walk_back(int leg_angle = stand_angle_leg)
{  
  for(int i =2;i<=6;i=i+2)  //Open leg 2,4,6
  {pwm.setPWM(motors[i][1], 0 ,sign_angle(open_angle_leg,i,1));}

  for(int _=0; _<shoulder_front_angle; _++)//Moves 1,3,5 to the front and 2,4,6 back and registers current positions
  {
    for(int j=1; j<6; j=j+2)
    {
      CurrentPosition[j][0] += orientation[j][0];  pwm.setPWM(motors[j][0], 0 ,CurrentPosition[j][0]);//1,3,5
      CurrentPosition[j+1][0] -= orientation[j+1][0];  pwm.setPWM(motors[j+1][0], 0 ,CurrentPosition[j+1][0]);//2,4,6
    }delay(speed);
  }
  
  for(int i =2;i<=6;i=i+2) //Close leg 2,4,6
  {pwm.setPWM(motors[i][1], 0 ,sign_angle(leg_angle,i,1));}


  delay(speed*4);
  for(int i =1;i<=6;i=i+2) //Open leg 1,3,5
  {pwm.setPWM(motors[i][1], 0 ,sign_angle(open_angle_leg,i,1));}

  for(int _=0; _<shoulder_front_angle*2; _++)//Moves 1,3,5 to the back and 2,4,6 front and registers current positions
  {
    for(int j=1; j<6; j=j+2)
    {
      CurrentPosition[j][0] -= orientation[j][0]; pwm.setPWM(motors[j][0], 0 ,CurrentPosition[j][0]);//1,3,5
      CurrentPosition[j+1][0] += orientation[j+1][0];  pwm.setPWM(motors[j+1][0], 0 ,CurrentPosition[j+1][0]);//2,4,6
    }delay(speed);
  }
  
  for(int i =1;i<=6;i=i+2) //Close leg 1,3,5
  {pwm.setPWM(motors[i][1], 0 ,sign_angle(leg_angle,i,1));}

  delay(speed*4);
  for(int i =2;i<=6;i=i+2)  //Open leg 2,4,6
  {pwm.setPWM(motors[i][1], 0 ,sign_angle(open_angle_leg,i,1));}

  for(int _=0; _<shoulder_front_angle; _++)//Moves 1,3,5 to the front and 2,4,6 back and registers current positions
  {
    for(int j=1; j<6; j=j+2)
    {
      CurrentPosition[j][0] += orientation[j][0];  pwm.setPWM(motors[j][0], 0 ,CurrentPosition[j][0]);//1,3,5
      CurrentPosition[j+1][0] -= orientation[j+1][0];  pwm.setPWM(motors[j+1][0], 0 ,CurrentPosition[j+1][0]);//2,4,6
    }delay(speed);
  }

}


void turn_right(int leg_angle = stand_angle_leg)
{
  // Delay to control the speed of the turn
  delay(TurnSpeed*8);
  
  // Open legs 2, 6, and 4
  // Set the servo angles to the open angle for the corresponding legs
  pwm.setPWM(motors[2][1], 0 , sign_angle(open_angle_leg,2,1)); 
  pwm.setPWM(motors[6][1], 0 , sign_angle(open_angle_leg,6,1));
  pwm.setPWM(motors[4][1], 0 , sign_angle(open_angle_leg,4,1));
  
  // Move legs 1 and 5 backward for the specified shoulder turn angle
  // Decrease the servo angles by the orientation value for the corresponding legs
  for(int i=0; i<shoulder_turn_angle;i++)
  {
    CurrentPosition[1][0]-=orientation[1][0];  
    pwm.setPWM(motors[1][0], 0 , CurrentPosition[1][0]); 

    CurrentPosition[5][0]-=orientation[5][0];  
    pwm.setPWM(motors[5][0], 0 , CurrentPosition[5][0]);
    delay(TurnSpeed);
  }
  
  // Close legs 2, 6, and 4
  // Set the servo angles to the specified leg angle for the corresponding legs
  pwm.setPWM(motors[2][1], 0 , sign_angle(leg_angle,2,1)); 
  pwm.setPWM(motors[6][1], 0 , sign_angle(leg_angle,6,1));
  pwm.setPWM(motors[4][1], 0 , sign_angle(leg_angle,4,1));

  // Delay to allow time for the legs to close
  delay(TurnSpeed*8);

  // Open legs 1 and 5
  // Set the servo angles to the open angle for the corresponding legs
  pwm.setPWM(motors[1][1], 0 , sign_angle(open_angle_leg,1,1)); 
  pwm.setPWM(motors[5][1], 0 , sign_angle(open_angle_leg,5,1));
  
  // Move legs 1 and 5 forward for the specified shoulder turn angle
  // Increase the servo angles by the orientation value for the corresponding legs
  for(int i=0; i<shoulder_turn_angle;i++)
  {
    CurrentPosition[1][0]+=orientation[1][0];  
    pwm.setPWM(motors[1][0], 0 , CurrentPosition[1][0]); 

    CurrentPosition[5][0]+=orientation[5][0];  
    pwm.setPWM(motors[5][0], 0 , CurrentPosition[5][0]);
    delay(TurnSpeed);
  }
  
  // Close legs 1 and 5
  // Set the servo angles to the specified leg angle for the corresponding legs
  pwm.setPWM(motors[1][1], 0 , sign_angle(leg_angle,1,1)); 
  pwm.setPWM(motors[5][1], 0 , sign_angle(leg_angle,5,1));
}

void turn_left(int leg_angle = stand_angle_leg)
{
  delay(TurnSpeed*8);

  // Open leg angles for motors 2 and 6
  pwm.setPWM(motors[2][1], 0, sign_angle(open_angle_leg,2,1)); 
  pwm.setPWM(motors[6][1], 0, sign_angle(open_angle_leg,6,1));

  // Forward movement for motors 2 and 6
  for(int i=0; i<shoulder_turn_angle;i++)
  {
    // Update current position for motors 2 and 6 based on orientation
    CurrentPosition[2][0]+=orientation[2][0];  
    pwm.setPWM(motors[2][0], 0, CurrentPosition[2][0]); 

    CurrentPosition[6][0]+=orientation[6][0];  
    pwm.setPWM(motors[6][0], 0, CurrentPosition[6][0]);
    
    delay(TurnSpeed);
  }

  // Close leg angles for motors 2 and 6
  pwm.setPWM(motors[2][1], 0, sign_angle(leg_angle,2,1)); 
  pwm.setPWM(motors[6][1], 0, sign_angle(leg_angle,6,1));
  delay(TurnSpeed*8);

  // Open leg angles for motors 1, 3, and 5
  pwm.setPWM(motors[1][1], 0, sign_angle(open_angle_leg,1,1)); 
  pwm.setPWM(motors[3][1], 0, sign_angle(open_angle_leg,3,1)); 
  pwm.setPWM(motors[5][1], 0, sign_angle(open_angle_leg,5,1));

  // Backward movement for motors 2 and 6
  for(int i=0; i<shoulder_turn_angle;i++)
  {
    // Update current position for motors 2 and 6 based on orientation
    CurrentPosition[2][0]-=orientation[2][0];  
    pwm.setPWM(motors[2][0], 0, CurrentPosition[2][0]); 

    CurrentPosition[6][0]-=orientation[6][0];  
    pwm.setPWM(motors[6][0], 0, CurrentPosition[6][0]);
    delay(TurnSpeed);
  }

  // Close leg angles for motors 1, 3, and 5
  pwm.setPWM(motors[1][1], 0, sign_angle(leg_angle,1,1)); 
  pwm.setPWM(motors[3][1], 0, sign_angle(leg_angle,3,1));
  pwm.setPWM(motors[5][1], 0, sign_angle(leg_angle,5,1));
}


void front_right(int leg_angle = stand_angle_leg, int shoulder_front_angle_tmp = shoulder_front_angle)
{
  for(int i =1;i<=6;i=i+2) //Open leg 1,3,5
  {
    pwm.setPWM(motors[i][1], 0 , sign_angle(leg_angle+leg_open_plus,i,1));
    }

    for(int _=0; _<shoulder_front_angle_tmp; _++)//Moves 1,3,5 to the front and 2,4,6 back and registers current positions
  {
      CurrentPosition[1][0] = CurrentPosition[1][0] + 2*orientation[1][0];  
      pwm.setPWM(motors[1][0], 0 , CurrentPosition[1][0]);
      
      CurrentPosition[5][0] = CurrentPosition[5][0] + 2*orientation[5][0]; 
      pwm.setPWM(motors[5][0], 0 , CurrentPosition[5][0]);
      
      CurrentPosition[3][0] = CurrentPosition[3][0] + orientation[3][0]; 
      pwm.setPWM(motors[3][0], 0 , CurrentPosition[3][0]);



      CurrentPosition[2][0] = CurrentPosition[2][0] - orientation[2][0];  
      pwm.setPWM(motors[2][0], 0 , CurrentPosition[2][0]);
      
      CurrentPosition[4][0] = CurrentPosition[4][0] - 2*orientation[4][0]; 
      pwm.setPWM(motors[4][0], 0 , CurrentPosition[4][0]);

      CurrentPosition[6][0] = CurrentPosition[6][0] - orientation[6][0]; 
      pwm.setPWM(motors[6][0], 0 , CurrentPosition[6][0]);
    delay(speed);
  }

  for(int i =1;i<=6;i=i+2) //Close leg 1,3,5
  {
    pwm.setPWM(motors[i][1], 0 , sign_angle(leg_angle,i,1));
  }
  
  
  delay(speed*4);

  for(int i =2;i<=6;i=i+2)  //Open leg 2,4,6
  {
    pwm.setPWM(motors[i][1], 0 , sign_angle(leg_angle+leg_open_plus,i,1));
    }

  for(int _=0; _<shoulder_front_angle_tmp*2; _++)//Moves 1,3,5 to the front and 2,4,6 back and registers current positions
  {
      CurrentPosition[1][0] = CurrentPosition[1][0] - 2*orientation[1][0];  
      pwm.setPWM(motors[1][0], 0 , CurrentPosition[1][0]);
      
      CurrentPosition[5][0] = CurrentPosition[5][0] - 2*orientation[5][0]; 
      pwm.setPWM(motors[5][0], 0 , CurrentPosition[5][0]);
      
      CurrentPosition[3][0] = CurrentPosition[3][0] - orientation[3][0]; 
      pwm.setPWM(motors[3][0], 0 , CurrentPosition[3][0]);



      CurrentPosition[2][0] = CurrentPosition[2][0] + orientation[2][0];  
      pwm.setPWM(motors[2][0], 0 , CurrentPosition[2][0]);
      
      CurrentPosition[4][0] = CurrentPosition[4][0] + 2*orientation[4][0]; 
      pwm.setPWM(motors[4][0], 0 , CurrentPosition[4][0]);

      CurrentPosition[6][0] = CurrentPosition[6][0] + orientation[6][0]; 
      pwm.setPWM(motors[6][0], 0 , CurrentPosition[6][0]);
    delay(speed);
  }



  for(int i =2;i<=6;i=i+2) //Close leg 2,4,6
  {
    pwm.setPWM(motors[i][1], 0 , sign_angle(leg_angle,i,1));
    }


  delay(speed*4);
  for(int i =1;i<=6;i=i+2) //Open leg 1,3,5
  { 
    pwm.setPWM(motors[i][1], 0 , sign_angle(leg_angle+leg_open_plus,i,1));}

  

      for(int _=0; _<shoulder_front_angle_tmp; _++)//Moves 1,3,5 to the front and 2,4,6 back and registers current positions
  {
      CurrentPosition[1][0] = CurrentPosition[1][0] + 2*orientation[1][0];  
      pwm.setPWM(motors[1][0], 0 , CurrentPosition[1][0]);
      
      CurrentPosition[5][0] = CurrentPosition[5][0] + 2*orientation[5][0]; 
      pwm.setPWM(motors[5][0], 0 , CurrentPosition[5][0]);
      
      CurrentPosition[3][0] = CurrentPosition[3][0] + orientation[3][0]; 
      pwm.setPWM(motors[3][0], 0 , CurrentPosition[3][0]);



      CurrentPosition[2][0] = CurrentPosition[2][0] - orientation[2][0];  
      pwm.setPWM(motors[2][0], 0 , CurrentPosition[2][0]);
      
      CurrentPosition[4][0] = CurrentPosition[4][0] - 2*orientation[4][0]; 
      pwm.setPWM(motors[4][0], 0 , CurrentPosition[4][0]);

      CurrentPosition[6][0] = CurrentPosition[6][0] - orientation[6][0]; 
      pwm.setPWM(motors[6][0], 0 , CurrentPosition[6][0]);
    delay(speed);
  }

}

void front_left(int leg_angle = stand_angle_leg, int shoulder_front_angle_tmp = shoulder_front_angle)
{
  for (int i = 2; i <= 6; i = i + 2) // Open leg 2, 4, 6
  {
    pwm.setPWM(motors[i][1], 0, sign_angle(leg_angle + leg_open_plus, i, 1));
  }

  for (int _ = 0; _ < shoulder_front_angle_tmp; _++) // Moves 2, 4, 6 to the front and 1, 3, 5 back and registers current positions
  {
    CurrentPosition[2][0] = CurrentPosition[2][0] + 2*orientation[2][0];
    pwm.setPWM(motors[2][0], 0, CurrentPosition[2][0]); // 2, 4, 6

    CurrentPosition[4][0] = CurrentPosition[4][0] +  orientation[4][0];
    pwm.setPWM(motors[4][0], 0, CurrentPosition[4][0]); // 2, 4, 6

    CurrentPosition[6][0] = CurrentPosition[6][0] + 2* orientation[6][0];
    pwm.setPWM(motors[6][0], 0, CurrentPosition[6][0]); // 2, 4, 6



    CurrentPosition[1][0] = CurrentPosition[1][0] - orientation[1][0];
    pwm.setPWM(motors[1][0], 0, CurrentPosition[1][0]); // 1, 3, 5

    CurrentPosition[3][0] = CurrentPosition[3][0] - 2*orientation[3][0];
    pwm.setPWM(motors[3][0], 0, CurrentPosition[3][0]); // 1, 3, 5

    CurrentPosition[5][0] = CurrentPosition[5][0] - orientation[5][0];
    pwm.setPWM(motors[5][0], 0, CurrentPosition[5][0]); // 1, 3, 5

    delay(speed);
  }

  for (int i = 2; i <= 6; i = i + 2) // Close leg 2, 4, 6
  {
    pwm.setPWM(motors[i][1], 0, sign_angle(leg_angle, i, 1));
  }

  delay(speed * 4);


  for (int i = 1; i <= 6; i = i + 2) // Open leg 1, 3, 5
  {
    pwm.setPWM(motors[i][1], 0, sign_angle(leg_angle + leg_open_plus, i, 1));
  }

  for (int _ = 0; _ < shoulder_front_angle_tmp*2; _++) // Moves 1,3, 5 to the front and 2, 4, 6 back and registers current positions
  {
    CurrentPosition[1][0] = CurrentPosition[1][0] + orientation[1][0];
    pwm.setPWM(motors[1][0], 0, CurrentPosition[1][0]); // 1, 4, 6

    CurrentPosition[3][0] = CurrentPosition[3][0] + 2 * orientation[3][0];
    pwm.setPWM(motors[3][0], 0, CurrentPosition[3][0]); // 1, 3, 6

    CurrentPosition[5][0] = CurrentPosition[5][0] + orientation[5][0];
    pwm.setPWM(motors[5][0], 0, CurrentPosition[5][0]); // 1, 3, 5

    CurrentPosition[2][0] = CurrentPosition[2][0] - 2* orientation[2][0];
    pwm.setPWM(motors[2][0], 0, CurrentPosition[2][0]); // 2, 3, 5

    CurrentPosition[4][0] = CurrentPosition[4][0] - orientation[4][0];
    pwm.setPWM(motors[4][0], 0, CurrentPosition[4][0]); // 1, 4, 5

    CurrentPosition[6][0] = CurrentPosition[6][0] - 2 * orientation[6][0];
    pwm.setPWM(motors[6][0], 0, CurrentPosition[6][0]); // 1, 4, 5

    delay(speed);
  }

  for (int i = 1; i <= 6; i = i + 2) // Close leg 1, 3, 5
  {
    pwm.setPWM(motors[i][1], 0, sign_angle(leg_angle, i, 1));
  }

  delay(speed * 4);

  for (int i = 2; i <= 6; i = i + 2) // Open leg 2, 4, 6
  {
    pwm.setPWM(motors[i][1], 0, sign_angle(leg_angle + leg_open_plus, i, 1));
  }

  for (int _ = 0; _ < shoulder_front_angle_tmp; _++) // Moves 2, 4, 6 to the front and 1, 3, 5 back and registers current positions
  {
    CurrentPosition[2][0] = CurrentPosition[2][0] + 2* orientation[2][0];
    pwm.setPWM(motors[2][0], 0, CurrentPosition[2][0]); // 2, 4, 6

    CurrentPosition[4][0] = CurrentPosition[4][0] + orientation[4][0];
    pwm.setPWM(motors[4][0], 0, CurrentPosition[4][0]); // 2, 4, 6

    CurrentPosition[6][0] = CurrentPosition[6][0] + 2* orientation[6][0];
    pwm.setPWM(motors[6][0], 0, CurrentPosition[6][0]); // 2, 4, 6

    CurrentPosition[1][0] = CurrentPosition[1][0] - orientation[1][0];
    pwm.setPWM(motors[1][0], 0, CurrentPosition[1][0]); // 1, 3, 5

    CurrentPosition[3][0] = CurrentPosition[3][0] - 2* orientation[3][0];
    pwm.setPWM(motors[3][0], 0, CurrentPosition[3][0]); // 1, 3, 5

    CurrentPosition[5][0] = CurrentPosition[5][0] - orientation[5][0];
    pwm.setPWM(motors[5][0], 0, CurrentPosition[5][0]); // 1, 3, 5

    delay(speed);
  }


}

void Go_down()
{
  for(int i =1;i<=6;i++) 
  {pwm.setPWM(motors[i][1], 0 , sign_angle(170,i,1));}
}


void commad_arms(int angle,int i)
{
  int x=1;int k;
  if (angle<current_Pos_arms[i])
  {x=-1;}


  for( k=current_Pos_arms[i]; k!= angle; k=k+x)
  {
    pwm.setPWM(arms[i], 0 , angleToPulseWidth_SG(k));
    delay(speed_arms);
  }
  current_Pos_arms[i]=k;
}



void setup() {

  Serial.begin(9600);
  // Initialize the PCA9685 motor driver
  pwm.begin();
  // Set the frequency to 50Hz
  pwm.setPWMFreq(50);  
  pinMode(7, OUTPUT);
  GoHome();
  pwm.setPWM(arms[0], 0 , angleToPulseWidth_SG(Initial_position_arms[0]));
  pwm.setPWM(arms[1], 0 , angleToPulseWidth_SG(Initial_position_arms[1]));
  digitalWrite(7, LOW);

}

  

void loop() {
    if (bimax.available())
    {old_command = command; command = bimax.read(); GoHome(); delay(400);Serial.println(command);}
    switch (command)
    {
        case 49:
          walk_front(); break;
        
        case 50:
          walk_back(); break;
        
        case 51:
          front_right(stand_angle_leg,17); break;
        
        case 52:
          front_left(stand_angle_leg,17); break;

        case 54:
          turn_right();
          break;
        
        case 53:
          turn_left();
          break;

        case 55:
          commad_arms(current_Pos_arms[0]+8,0);
          command = 500;
          break;
        
        case 56:
          commad_arms(current_Pos_arms[0]-8,0);
          command = 500;
          break;

        case 57:
          commad_arms(current_Pos_arms[1]+8,1);
          command = 500;
          break;
        
        case 65: //technacly A but it's a button 
          commad_arms(current_Pos_arms[1]-8,1);
          command = 500;
          break;

        case 32: //espace
          walk_front_low(120);
          break;
        
        case 104: //H
          digitalWrite(7, HIGH);
          delay(20);
          break;
        
        case 109: //M
          digitalWrite(7, LOW);
          delay(20);
          break;
        
        case 100: // d 
          Go_down();    
        
        case 500: // reference pour arms 
          break;
        
        default:
        GoHome(); 
        break;
    }
  

}
