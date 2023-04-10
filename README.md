
# Self Balancing bot



# Introduction

The aim of this project is to make an automated vehicle that balances itself without any outside help or support. This project is a rather complex one as it involves using PID Control and involuted programming. Self-balancing robots are unique among all others, just because of their ability to balance on a given fixed position. Even if the robot is displaced from its position, it is programmed so that it again recovers its position.

## Components

**MPU6050 :** MPU6050 is a three-axis accelerometer and three-axis gyroscope Micro Electro-mechanical system (MEMS). It aids in the measurement of velocity, orientation, acceleration, displacement, and other motion-related features.

**Arduino Uno :** Arduino UNO is a low-cost, flexible, and easy-to-use programmable open-source microcontroller board that can be integrated into a variety of electronic projects. This board can be interfaced with other Arduino boards, Arduino shields, Raspberry Pi boards and can control relays, LEDs, servos, and motors as an output.

**A4988 Motor driver :** The A4988 is a complete microstepping motor driver with built-in translator for easy operation. It is designed to operate bipolar stepper motors in full-, half-, quarter-, eighth-, and sixteenth-step modes, with an output drive capacity of up to 35 V and ±2 A.

**Stepper Motor :** Stepper motors are a type of DC synchronous motor. Whereas the rotation of an induction motor is largely uncontrollable, the rotation of a stepper motor can be controlled with a remarkable degree of precision. Stepper motors can produce full, instantaneous torque - even from a standstill.

**ESP32  :** ESP32 can perform as a complete standalone system or as a slave device to a host MCU, reducing communication stack overhead on the main application processor. ESP32 can interface with other systems to provide Wi-Fi and Bluetooth functionality through its SPI / SDIO or I2C / UART interfaces.

**Li polymer battery  :**  A lithium-polymer battery (LiPo) is a rechargeable battery that, in the case of true LiPo, uses solid polymer for the electrolyte and lithium for one of the electrodes. Commercially available LiPo are hybrids: gel polymer or liquid electrolyte in a pouch format, more accurately termed a lithium ion polymer battery.

**Jumper wire  :** Jumper wires typically come in three versions: male-to-male, male-to-female and female-to-female. The difference between each is in the end point of the wire. Male ends have a pin protruding and can plug into things, while female ends do not and are used to plug things into.

## Role of each software components-

**MPU6050 Data  :** Initially MPU-6050 will measure angle difference between current position of bot and its normal position . Now this data is transferred to PID.

**PID Tuning  :** PID will take input from MPU-6050 and accordingly calculate the PID value and give variable signals to both motors using interrupt service routine to bring the bot to its normal position and balance it .

**Bluetooth Communication  :** Once the bot is balanced then we will send different commands ( forward , backward , turn left , turn right and stop ) to the bot using bluetooth communication to move the bot in different directions.

**ISR Command  :** After receiving command from bluetooth , ISR will give different impulse to motors to move the bot according to the given command.

## Tech Stack

**On Robot:** Bluetooth , PID , TIMER Interrupt , Kalman Filter , Communication protocol (serial and I2C).

## Working of the bot

**1.** Turning the robot on.

**2.** The servo motor position is setted to minimum position to keep the robot in standing position.

**3.** Receiving the data from bluetooth.

**4.** Data is collected from the mpu-6050 and robot angle is calculated.

**5.** Data from mpu-6050 is filtered to remove the noise and make the data stable

**6.** Feeding the data in PID controller and error is calculated by comparing it with the bot’s current angle and balance point is delivered as output from PID controller which will be in a linear data form.

**7.** Converting the linear PID balance point into logarithmic balance point for running the stepper motor at variable speed.

**8.** Controlling the left and right motor separately.

**9.** **a** -->For controlling the left and right turn we changed the balance point of PID  for both motors separately.

**b** -->For moving forward and backward  we have changed the target angle where the robot has to balance.
ex.--  Forward→increased target angle 
       Backward→decreased target angle

**10.** Starting the loop again from 3rd point.



## Features

- The robot balances on two wheels.
- We can move the robot left,right,forward,backward.
- Option to set  the PID constants from the onboard knob as well as wirelessly from android application.
- The robot stands on its own by servo motor for calibration when powered on.
- We can reset the robot wirelessly in case it falls on the ground.
- Used PORT control of arduino for making the pin high and low as it is 60 times faster than digitalWrite function of arduino. 

## Circuit diagram

![App Screenshot](https://via.placeholder.com/468x300?text=App+Screenshot+Here)

![App Screenshot](https://via.placeholder.com/468x300?text=App+Screenshot+Here)

## Source Code

[Source Code](https://linktodocumentation)

## Video link

- https://youtu.be/Uv9UeYUsA8A
## Resources

 - Stepper motor control
   - Theoy
     - https://youtu.be/09Mpkjcr0bo
     - https://youtu.be/eyqwLiowZiU
   - Programming
     - https://www.youtube.com/watch?v=7spK_BkMJys
 - Arduino
   - https://youtube.com/playlist?list=PLGs0VKk2DiYw-L-RibttcvK-WBZm8WLEP

- MPU6050
  - https://youtu.be/XCyRXMvVSCw
  - https://www.youtube.com/watch?v=7VW_XVbtu9k
  - https://youtu.be/yhz3bRQLvBY
  - https://youtu.be/Yh6mYF3VdFQ

- Battery
  - https://www.youtube.com/watch?v=Tye3dcBOqtY

- Servo motor
  - https://youtu.be/kUHmYKWwuWs

- HC-05 bluetooth module
  - https://www.youtube.com/watch?v=yyjAAbxr0gM

- PID Controller
  - https://youtu.be/crw0Hcc67RY
  - https://youtu.be/t7ImNDOQIzM
  
- Communication 
  - https://youtu.be/IyGwvGzrqp8
  - https://youtu.be/fwGdmU2wi9s

- Filter
  - https://youtu.be/5HuN9iL-zxU

- Interrupt
  - https://youtu.be/Uv9UeYUsA8A

## Screenshots

![App Screenshot](https://via.placeholder.com/468x300?text=App+Screenshot+Here)


## Tech Stack

**On Robot:** Bluetooth , PID , TIMER Interrupt , Kalman Filter , Communication protocol (serial and I2C).

## Working of the bot







## Real-life applications

- Self balancing algorithms can be used in rocket propulsion systems.
- Can be used in warehouses for moving goods.
- It can be implemented on Segway.
- It can be implemented on Segway.


## Problem faced

**1.** MPU-6050 data has a lot of noise so we have a Kalman filter to smooth the data.

**2.** The setting of PID value was frustrating when setted  in code each time and checking the robot whether it is balanced or not. so we made three knobs to adjust the PID constant onboard without changing it in code and we also added this feature in the app so that the PID constant can also be setted wirelessly.

**3.** Bluetooth connection gets lost regularly afterward we figured out that the input voltage was under 5v . So we powered the ESP32 directly with a battery with 11v.

**4.** When potentiometer(for setting the PID constants) was powered with a 7805 voltage regulator(for 5v output) the robot oscillates  as the servos  were also connected to the 7805 and it drew a lot of power which made it undervolt(less than 5v)  and the value of constants gets changed when mapped in arduino code. So we powered it directly with 5v of arduino to get stable 5v.

**5.** Earlier we have used TIMER1 interrupt (ISR) but later realized that it was causing the problem as servo library we have used also use TIMER1 to make PWM signal and delay function in arduino uses TIMER0 for delay function so we have used the this arduino ISR i.e TIMER2 for creating interrupt for making Stepper motor impulse.

**7.** Stepper motor was not turning at very high speed so we have chosen an ISR of 200us as full speed for the stepper motor.

**8.** The bot was getting resetted in the setup  part of code in  which we had defined D13 as output and didn't work so we had defined the D13 as output in a “IF statement” in the loop part of code.

**9.** To prevent the potentiometer and app PID constant clashing with the original PID constant which we have setted in code we have setted a range so that we can deactivate the Potentiometer  and app PID constant values when not needed and activate it only When the values Kp & Kd exceeds 10 and Ki exceeds 2.

**10.** Used PORT control of arduino for making the pin high and low as it is 60 times faster than digitalWrite function of arduino and we have used it in ISR to make the impulse for driving the Stepper motor.

**11.** Detached the servo by software after running the main loop 200 times to prevent the oscillation of sevo and also prevent it further from drawing power from battery.

**12.** Converted the linear output of PID value to non linear i.e in Logarithmic value to run the stepper motor at different speed at different angles.

**13.** Controlled left and right Stepper motor separately for controlling the movement of the robot.

**14.** Made an Android app for controlling the movement of robots.

**15.** Separated the data coming from the app via bluetooth in string form and stored it in different variables.

**16.** The PID error of Ki was getting very large and affected the responsiveness of the robot so we had limited it in range (-200 to 200).

**17.** There was a problem with the app that it was not showing any bluetooth devices. Later we found that for connecting it with bluetooth it is first  necessary to ask by app for accessing the phone’s bluetooth. So we added the code in the app to ask for permission for accessing the bluetooth from a user in MIT app inventor. 

Note-This issue will only be faced in Android 9+ devices

**18.** Microstepping was used for smoother operation of steppe motor.

**19.** Always disconnect the Rx and Tx pin of the arduino while uploading the code to arduino.
