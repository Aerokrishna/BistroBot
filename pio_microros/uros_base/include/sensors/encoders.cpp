void updateEncoder(uint gpio, uint32_t events){

    if (gpio == enc_right_A){ 
      if (digitalRead(enc_right_B)==1){
        enc_val_right++;

      }
      else {
          enc_val_right--;}
      }
    
  
    if (gpio == enc_left_A){ 
      if (digitalRead(enc_left_B)==1){
        enc_val_left--;
  
      }
      else {
          enc_val_left++;}
      }
    // Serial.println("Hi");
  }