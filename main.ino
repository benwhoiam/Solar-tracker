#include <Servo.h>

// Constants
Servo upper;                           // Servo object for upper servo
Servo base;                            // Servo object for base servo
const int theta_initial = 180;         // Initial angle for theta (upper servo)
const int phi_initial = 90;            // Initial angle for phi (base servo)
const int theta_min = 15;              // Minimum angle for theta
const int theta_max = 180;             // Maximum angle for theta
const int phi_min = 0;                 // Minimum angle for phi
const int phi_max = 180;               // Maximum angle for phi
const int angle_increment = 3;         // Angle increment for servo movement
const int error_threshold = 10;        // Error threshold for LDR readings
const int delay_duration = 5;          // Delay between servo movements
const int correct_LDR_value = 7;       // Correction value for LDR readings
const int correct_LDR_min = 2;         // Minimum LDR reading for correction

// Variables
int LDR_values[4] = {0, 0, 0, 0};      // Array to store LDR readings
int theta = theta_initial;             // Current angle for theta (upper servo)
int phi = phi_initial;                 // Current angle for phi (base servo)
int Moy_LDR_up[2] = {0, 0};            // Array to store averaged LDR readings for upper direction
int Moy_LDR_down[2] = {0, 0};          // Array to store averaged LDR readings for lower direction
int checker;                           // Variable to determine angle increment direction

// Functions

void Command_motors() {   
    upper.write(theta);                // Set angle for theta (upper servo)
    base.write(phi);                   // Set angle for phi (base servo)
}

void Read_LDR_Values() {
    LDR_values[0] = analogRead(A3);    // Read LDR value for upper left direction
    LDR_values[1] = analogRead(A2);    // Read LDR value for upper right direction
    LDR_values[2] = analogRead(A0);    // Read LDR value for lower left direction
    LDR_values[3] = analogRead(A1);    // Read LDR value for lower right direction
}

void Serial_Print_Values() {
    Serial.print("\n\n\n\n");
    Serial.print(LDR_values[0]); Serial.print(" || "); Serial.print(LDR_values[1]);
    Serial.print("\n");
    Serial.print(LDR_values[2]); Serial.print(" || "); Serial.print(LDR_values[3]);
    Serial.print("\n\n");
}

void Correct_LDR_Error() {
    if (LDR_values[3] >= correct_LDR_min) {
        LDR_values[3] += correct_LDR_value;  // Correct LDR value for lower right direction
    }
}

void Limits() {
    if (phi <= phi_min) { phi = phi_min; }               // Check and set limits for phi (base servo)
    else if (phi >= phi_max) { phi = phi_max; }

    if (theta <= theta_min) { theta = theta_min; }       // Check and set limits for theta (upper servo)
    else if (theta >= theta_max) { theta = theta_max; }
}

void setup() {
    Serial.begin(9600);                  // Initialize serial communication
    upper.attach(10);                    // Attach upper servo to pin 10
    base.attach(3);                      // Attach base servo to pin 3
    Command_motors();                    // Initialize servo angles to initial values
}

void loop() {
    Read_LDR_Values();                                    // Read LDR values
    Serial_Print_Values();                                // Print LDR values for debugging
    Correct_LDR_Error();                                  // Correct LDR error for lower right direction

    Moy_LDR_up[0] = (LDR_values[0] + LDR_values[1]) / 2;  // Calculate average LDR value for upper pair
    Moy_LDR_up[1] = (LDR_values[2] + LDR_values[3]) / 2;  // Calculate average LDR value for lower pair
    Moy_LDR_down[0] = (LDR_values[0] + LDR_values[2]) / 2; // Calculate average LDR value for left pair
    Moy_LDR_down[1] = (LDR_values[1] + LDR_values[3]) / 2; // Calculate average LDR value for right pair

    if (theta <= 90) {                                    // Check servo angle theta
        checker = angle_increment;                        // Set checker to positive angle increment
    } else if (theta > 90) {
        checker = -angle_increment;                       // Set checker to negative angle increment
    }

    if (abs(Moy_LDR_up[1] - Moy_LDR_up[0]) > error_threshold) {  // Check if there is a significant difference in LDR values for upper direction
        if (Moy_LDR_up[0] - Moy_LDR_up[1] < 0) {                   // If average LDR value for upper direction has decreased
            theta = theta + angle_increment;                      // Increase theta angle by the angle increment
        } else if (Moy_LDR_up[0] - Moy_LDR_up[1] > 0) {           // If average LDR value for upper direction has increased
            theta = theta - angle_increment;                      // Decrease theta angle by the angle increment
        }
    }

    if (abs(Moy_LDR_down[1] - Moy_LDR_down[0]) > error_threshold) { // Check if there is a significant difference in LDR values for lower direction
        if (Moy_LDR_down[0] - Moy_LDR_down[1] < 0) {                // If average LDR value for lower direction has decreased
            phi = phi - checker;                                    // Decrease phi angle by the checker value
        } else if (Moy_LDR_down[0] - Moy_LDR_down[1] > 0) {         // If average LDR value for lower direction has increased
            phi = phi + checker;                                    // Increase phi angle by the checker value
        }
    }

    Limits();           // Check and set limits for servo angles
    Command_motors();   // Command servo motors to move to new angles
    delay(delay_duration); // Delay for stability
}
