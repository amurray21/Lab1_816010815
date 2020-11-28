3006 Lab 1 Repository
For each exercise, ensure that the output file, the appropriately commented source files, and the final binary files, are part of your github repository.

Create a new project based on the ESP8266 SDK example titled "hello_world", and ensure that the code is sync'd with github. Adjust the example so that:
Replace the "hello world" message with a message that includes your name and ID#.
Replace the number of seconds to restart with the last two digits of your ID# modulo 15. e.g. for ID#8169875234 the number of seconds would be 34 modulo 15 = 4.
Capture the output of this program into a file named "lab1_q1_xxxxxxxxx.out" where xxxxxxxxx is your ID#
Create a new project based on the RTOS SDK example titled "gpio". This example is setup to use 4 pins, 2 inputs and 2 outputs. Adjust the example so that:
Only a single input and a single output are specified, and the input and output pins selected consistent with available pins on the ESP-01S
A single handler is installed and used for the input on the falling edge
A sempahore is passed from the ISR instead of a message
Capture the output of this program into a file named "lab1_q2_xxxxxxxxx.out" where xxxxxxxxx is your ID# Create a new project based on the RTOS SDK example titled "i2c". This example is setup to use the MPU6050 - adjust the example to use the ADS1115 by:
Configure the #defines associated with the SCL and SDA lines to be consistent with available pins on the ESP-01S and verify that the master_init routine is appropriate.
Remove the #defines associated with the MPU6050, determine the I2C address of the ADS1115 and the ADS1115 registers to be configured/read/written on the ADS1115 to allow a single-sided analog input (with no explicit voltage reference lines) on channel 0, create new #defines and adjust the master_device_read, master_device_write, and master_device_init routines accordingly.
Replace i2c_task_example with an example that will read and report the analog signal connected to channel 0 of the ADC at appropriate intervals.
Capture the output of this program into a file named "lab1_q3_xxxxxxxxx.out" where xxxxxxxxx is your ID#
