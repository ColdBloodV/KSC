# GND Station Operation Guide

## Overview
T-SAT A Ground Station is responsible for: 
  * Sending the 'Detach' message to the T-SAT
  * Receiving altitude data from the T-SAT
  * Receiving the 'Detached' ACK from the T-SAT
    
---

## Detach Button Operation 
The Detach button can only be triggered **once every 30 seconds**.

After the button is pressed:
1. The 'Detach' message is sent to the T-SAT
2. The GND station waits **25 seconds** for a reply
3. During this time the T-SAT:
   - Finishes saving any images
   - Triggers the servo
   - Sends back the 'Detached' ACK

After the 'Detached' ACK is recevied the 30 second cooldown will begin. 

---

## Important Timing Notes

**Do NOT send the Detach command immediately after powering on the T-SAT.**

**Do NOT press the Detach button immediately after the 30 second cooldown ends.**
Wait a few seconds before pressing again.

Failure to do this may cause the system to freeze.

---

## Reasons for Freeze Issue
While the cooldown timer is active, the GND station is not receving packets. During this time, altitude packets from the T-SAT may queue up and only seen after the cooldown so you might get more than 1 altitude reading afterwards.

Once the cooldown finishes: 
- Multiple altitude packets may be received at once
- If the Ground Station tries to send the 'Detach' command at the same time it is receiving packets, the LoRa module can freeze.

The LoRa RF module can **only send OR receive at one time**, not both simultaneously.

It is best to just wait a couple seconds and then send the 'Detach' message again.

This issue was handled with Timers and a flush packets function but the timing isn't 100% accurate. 


---

## Timing Summary

| Event | Time |
|------|------|
| Button Cooldown | 30 seconds |
| Wait for ACK | 25 seconds |
| Recommended Wait After Cooldown | 2–5 seconds |
| Servo Activation Delay (T-SAT) | 15–25 seconds |
| Altitude Transmission Interval | 25 seconds |


