//CONTROLLER INPUT
    throttle=map(CH3, 1065, 1900, 0, 127);
    steering=map(CH1, 1065, 1900, -127, 127);
    //Throttle: postive forward / negative reverse.  Steering: positive left / negative right.
    //In this example throttle is mapped for the full motion of the stick to be forward, for more precision.
    //For forward/reverse on same stick (center neutral), you'd map -127 to +127 instead.

    //INERTIA
    //savedthrottle variable is initialized as 0
    //steermargin is set to 3 (out of 127).
    //The lower the number, the less input variation is required to trigger "smoothing" of the input
    if(abs(throttle-savedthrottle)>steermargin){
      if(throttle>savedthrottle){
        throttle=savedthrottle+steermargin;
        savedthrottle=throttle;
      } else {
        throttle=savedthrottle-steermargin;
        savedthrottle=throttle;
      }
    } else {
      savedthrottle=throttle;
    }

    throttle=abs(throttle);

    //Map the steering min/max so that it is no greater than the throttle input
    //In other words, the steering is proportional with the throttle
    steermap=map(steering, -127, 127, throttle, -throttle);
    steering=abs(steermap);


    //Set steering left or right, to determine which motor to "brake" in next step
      if(steering<0){
        steerswitch=1;
      }else{
        steerswitch=-1;
      }

    //Depending on left or right, pick which motor to modify.  The other gets the straight throttle input.
    //Inside track braking is simulated by subtracting steering input from the throttle of the inside track.
    //dirswitch is the forward/reverse switch (+1/-1).
      if(steerswitch==1){
        ST.motor(2, (throttle-steering)*dirswitch);
        ST.motor(1, throttle*dirswitch);
      }else{
        ST.motor(1, (throttle-steering)*dirswitch);
        ST.motor(2, throttle*dirswitch);
      }
