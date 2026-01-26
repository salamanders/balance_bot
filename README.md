# balance_bot

Wiring Check:

* MPU-6050: VCC to 3.3V, GND to GND, SDA to SDA, SCL to SCL.
* Motors: Plugged into Motor A and Motor B on the Picon Zero.

Orientation: This code assumes the MPU is mounted such that tilting forward/back rotates the X-axis gyroscope. If your robot acts crazy immediately, change data['gx'] to data['gy'] in the get_pitch method.

Bootup: 

1. Run python3 balance_bot.py
2. Hold the robot perfectly upright and still. The console will say [STEP 1] HOLD VERTICAL. Wait 1 second.
3. The console will say [STEP 2] LETTING IT WOBBLE. Gently support it with your hands (don't grip it tight, just cage it). It will twitch back and forth.
4. The console will print Tuned! Kp: ... and it will take over. Let go!

