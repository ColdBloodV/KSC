How to Operate GND station

The Button to send the 'detach' message and activate the Servo can only be triggered once and theres a cool down of thirty seconds before it can be pressed again. 

Once the 'Detach' message is sent to the T-SAT GND station will wait for 25 seconds for a reply giving it enough time for the T-SAT to finish any Picture saving, Trigger the servo, and reply back. 

* IMPORTANT

While Testing noticed that sending the ACK ('Detach') packet right after turning on the T-SAT will cause a freeze. Also sending an ACK right after the cooldown finishes can also trigger a freeze, 
so it is advised to wait a couple seconds. 

This is because while the cooldown is happening we are not receving any packets so the altitude messages from T-SAT get queued and only seen after the cooldown so you might get more than 1 altitude reading afterwards.
It is best to just wait a couple seconds and then send the 'Detach' message again. Since the LORA RF module can only send or receive data at a time if we send or receive at the same time it will freeze. 

This issue was handled with Timers but the timing isn't 100% guarenteed. 
