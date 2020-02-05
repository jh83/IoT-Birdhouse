# IoT-BirdHouse - Readme under development

My first IoT project based on LoRa.

![Image of Birdhouse](images/Inside.jpg?raw=true)

The goal with this project was to have a solution to monitor the amount of birdseed left in our bird feeder. We noticed that on some days,the birds appeard to be really hungry resulting in an empty feeder within a couple of days after refill.

I decided on a solution to measure the amount if seed with a "time of flight" VL6180X which can measure up to 100mm with mm accuracy. Similar modules exists that can measure longer distances if needed.

The Vl6180X is very small in size and uses laser (not ultrasonic sound which is more common way of measuring distance) and therefore fits this use case.

A SHT31-D temperature/humidity sensor is used for making accurate readings of temp/hum (as accurate as those can be when placed inside a wooden box). This module has a accuracy of ±2% relative humidity and ±0.3°C.

The "brain" is a Arduino Pro Mini which has been modified by removing the onboard to use less current during sleep.

Current used for Voltage measurement is reduced by having a mosfet active only when voltage measurement is done.

A RFM95 chip is used for LoRa transmissions. If i would redo this project today, then i would have used a certified LoRa chip like the RAK Wireless ones or similar.

The Arduino Pro Mini wakes up every 5 minutes, performs a temperature/humidity and a "distance" measurement. It captures three of those and then on the third (after 15 minutes) it sends it via LoRa to TheThingsNetwork where its then later sent for further visualization.

The hardware along with a 18650 battery is mounted in a 3D printed plastic box.

Today (2020-02-05) this devices has been up and running since june 2018 on a single battery charge and it still reports 3.2Volts so the current draw is very small.