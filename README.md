
1. Motor inversion moved: the file now has MOTOR_INVERT = {false,false,false,false} and ENC_INVERT = {true,false,true,false}. These do different jobs. MOTOR_INVERT makes positive duty roll the wheel forward. ENC_INVERT makes a wheel rolling forward count positive. If motors 1 and 3 really roll backward on positive duty, the encoder flip hides that from the loop, so the loop is happy while the wheels are physically reversed. Forward would then spin the robot instead of driving it, unless you wired around it.
2. Wheel order: your inverted pair is M1 and M3, which is one side of the robot if M1=FL and M3=RL. The Pi node sends [fl, fr, rr, rl] to M1..M4, meaning it assumes M3=RR. If M3 is really the rear-left, the rear wheels get each other's turn and strafe values.

Test (robot on blocks, wheels free): send LOOP,0, then MOTOR,50,0,0,0, then STOP. Repeat with the 50 in position 2, 3 and 4. For each motor, tell me:
- which corner spins (front-left, front-right, rear-left or rear-right),
- whether it rolls toward the robot's front or its back,
- the sign of that wheel's count in the # ENC counts line.
