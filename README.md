In this program, we designed and implemented a traffic light system simulator for traffic control using Arm Keil MDK, STM32Cube MX and Proteus software and with the help of ARM microcontroller STM32F401VC series.

This system includes a LCD character, six LEDs, a four-digit 7-segment and six keys to control the traffic light.
Rules:
1. Six LEDs, each corresponding to one color of the traffic light, and each of them will be turned on at the proper time according to the state of the light.
2. There are two traffic lights in this intersection. The current state of the lights, which is their color, is displayed on the LCD.
    - For example, when the first traffic light is red, the word "First traffic light is RED" is displayed on the LCD. 
3. At the same time, the state of the second traffic light is also displayed on the lower line on the LCD.
4. The duration of each color is determined by a hardware timer. 
    - 20 seconds for red light
    - 5 seconds for yellow 
    - 15 seconds for green
5. The total time of yellow and green is equal to red, so according to the rules, whenever one side of the intersection is red, the other side turns green and yellow. The time each light is on is displayed on the four-digit 7-segment.
6. Each of the two digits of 7-segment is for one of the intersection lights. 
    - Two digits on the left -> for the first traffic light 
    - Two digits on the right -> for the second traffic light
7. With six external keys, we can change the state of the lights regardless of the remaining time. Each key is for one color of traffic light.
8. Besides the keys, we can change the state of the lights by sending special commands using UART.
